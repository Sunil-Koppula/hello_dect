/*
 * Mesh protocol shared utilities for DECT NR+ mesh network
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include "mesh.h"
#include "main_sub.h"
#include "tracker.h"
#include "radio.h"
#include "queue.h"
#include "storage.h"
#include "protocol.h"
#include "product_info.h"
#include "data.h"
#include "log_color.h"

LOG_MODULE_REGISTER(mesh, CONFIG_MESH_LOG_LEVEL);

#include "timeout.h"

#define COLLECT_WINDOW_MS     3000
#define MAX_RESPONSE_CANDIDATES 16

/* Candidate from a PAIR_RESPONSE during collection. */
struct response_candidate {
    uint16_t sender_id;
    uint8_t  device_type;
    uint8_t  hop_num;
    int16_t  rssi_2;
};

static struct response_candidate resp_candidates[MAX_RESPONSE_CANDIDATES];
static int resp_candidate_count;
static int resp_candidate_idx = 0;
static struct nbtimeout collect_timer;
static bool collecting;
static bool process_next_request = false;

/* Check Infra Storage */
static uint8_t check_infra_storage(uint16_t device_id, uint8_t device_type, bool all_slots)
{
    // Check Only first slot as sensor can only pair with one gateway/anchor, but anchor can pair with multiple sensors
    if (all_slots == false) {
        if (infra_count >= 1 && infra_devices[0].entry.device_id == device_id) {
            return STATUS_ALREADY_EXISTS;
        }

        if (infra_count >= 1) {
            return STATUS_STORAGE_FULL;
        }

        return STATUS_SUCCESS;
    }

    for (int i = 0; i < infra_count; i++) {
        if (infra_devices[i].entry.device_id == device_id) {
            return STATUS_ALREADY_EXISTS;
        }
    }

    if (infra_count >= MAX_ANCHORS) {
        return STATUS_STORAGE_FULL;
    }

    return STATUS_SUCCESS;
}

/* Update Infra Storage */
static bool update_infra_storage(uint16_t device_id, uint8_t hop_num, int16_t rssi_2)
{
    uint8_t current_hop_num = DEVICE_HOP_NUMBER;
    infra_entry_t entry;
    for (int i = 0; i < infra_count; i++) {
        if (infra_devices[i].entry.device_id == device_id) {
            int err = storage_infra_get(i, &entry);
            if (err) {
                LOG_ERR("Failed to get infra entry for device %d, err %d", device_id, err);
                return false;
            }
            entry.rssi_2 = rssi_2;
            if (hop_num != 0xFF && entry.hop_num != hop_num) {
                entry.hop_num = hop_num;
            }
            if (entry.version != FIRMWARE_VERSION) {
                entry.version = FIRMWARE_VERSION;
            }
            err = storage_infra_update(i, &entry);
            if (err) {
                LOG_ERR("Failed to update infra entry for device %d, err %d", device_id, err);
                return false;
            }
            device_info_update();
            break;
        }
    }
    if (current_hop_num != DEVICE_HOP_NUMBER) {
        return true;
    }
    return false;
}

/* Check Sensor Storage */
static uint8_t check_sensor_storage(uint16_t device_id)
{
    for (int i = 0; i < sensor_count; i++) {
        if (sensor_devices[i].entry.device_id == device_id) {
            return STATUS_ALREADY_EXISTS;
        }
    }

    if (sensor_count >= MAX_SENSORS) {
        return STATUS_STORAGE_FULL;
    }

    return STATUS_SUCCESS;
}

/* Update Sensor Storage */
static void update_sensor_storage(uint16_t device_id, uint16_t version)
{
    sensor_entry_t entry;
    for (int i = 0; i < sensor_count; i++) {
        if (sensor_devices[i].entry.device_id == device_id) {
            int err = storage_sensor_get(i, &entry);
            if (err) {
                LOG_ERR("Failed to get sensor entry for device %d, err %d", device_id, err);
                return;
            }
            if (entry.version == version) {
                return;
            }
            entry.version = version;
            err = storage_sensor_update(i, &entry);
            if (err) {
                LOG_ERR("Failed to update sensor entry for device %d, err %d", device_id, err);
            }
            break;
        }
    }
}

/* Check Mesh Storage */
static uint8_t check_mesh_storage(uint16_t device_id)
{
    mesh_entry_t entry;

    for (int i = 0; i < storage_mesh_count(); i++) {
        if (storage_mesh_get(i, &entry) == 0 && entry.device_id == device_id) {
            return STATUS_ALREADY_EXISTS;
        }
    }

    if (storage_mesh_count() >= MAX_DEVICES) {
        return STATUS_STORAGE_FULL;
    }

    return STATUS_SUCCESS;
}

/* Update Mesh Storage */
static void update_mesh_storage(uint16_t device_id, uint8_t hop_num, uint16_t version, uint16_t connected_device_id, uint8_t sensor_count)
{
    mesh_entry_t entry;
    for (int i = 0; i < storage_mesh_count(); i++) {
        if (storage_mesh_get(i, &entry) == 0 && entry.device_id == device_id) {
            if (entry.hop_num != hop_num && hop_num != 0xFF) {
                entry.hop_num = hop_num;
            }
            if (entry.version != version) {
                entry.version = version;
            }
            if (entry.connected_device_id != connected_device_id && connected_device_id != 0xFFFF) {
                entry.connected_device_id = connected_device_id;
            }
            if (entry.sensor_count != sensor_count && sensor_count != 0xFF) {
                entry.sensor_count = sensor_count;
            }
            int err = storage_mesh_update(i, &entry);
            if (err) {
                LOG_ERR("Failed to update mesh entry for device %d, err %d", device_id, err);
            }
            break;
        }
    }
}

/* Simple bubble sort candidates by hop (ascending), then RSSI (descending). */
static void sort_candidates(void)
{
    for (int i = 0; i < resp_candidate_count - 1; i++) {
        for (int j = 0; j < resp_candidate_count - i - 1; j++) {
            bool swap = false;

            if (resp_candidates[j].hop_num > resp_candidates[j + 1].hop_num) {
                swap = true;
            } else if (resp_candidates[j].hop_num == resp_candidates[j + 1].hop_num &&
                       resp_candidates[j].rssi_2 < resp_candidates[j + 1].rssi_2) {
                /* Same hop — prefer better (higher) RSSI. */
                swap = true;
            }

            if (swap) {
                struct response_candidate tmp = resp_candidates[j];
                resp_candidates[j] = resp_candidates[j + 1];
                resp_candidates[j + 1] = tmp;
            }
        }
    }
}

/* Select best candidates and send PAIR_CONFIRM to each. */
static void select_and_confirm(int idx)
{
    struct response_candidate *c = &resp_candidates[idx];

    /* Skip if already stored. */
    uint8_t status = check_infra_storage(c->sender_id, c->device_type, true);

    if (status == STATUS_STORAGE_FULL) {
        LOG_WRN("Infra storage full, stopping selection");
        resp_candidate_count = 0;
        return;
    }

    if (DEVICE_HOP_NUMBER == 0xFF && c->hop_num == 0xFF) {
        LOG_WRN("Candidate %s ID:%d has no upstream link or has invalid hop number %d, skipping", device_type_str(c->device_type), c->sender_id, c->hop_num);
        process_next_request = true;
        return;
    }

    send_pair_confirm(c->sender_id, c->device_type, STATUS_SUCCESS);

    if (DEVICE_TYPE == DEVICE_TYPE_SENSOR) {
        // Sensor only pairs with one anchor, so stop after the first confirmation
        resp_candidate_count = 0;
        return;
    }
}

bool mesh_is_collecting(void)
{
    return collecting;
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------**** Handlers Functions ****--------------------------------------------------------------------------- */

static void build_route_info_and_send(uint16_t dst_id, uint8_t dst_type, const route_info_t *route, int16_t rssi_2, bool is_called_from_pair)
{
    infra_entry_t entry;
    uint16_t next_hop_id[MAX_ANCHORS];;
    uint8_t next_hop_count = 0;
    uint16_t gateway_id = 0xFFFF;

    if (is_called_from_pair) {
        for (int i = 0; i < storage_infra_count(); i++) {
            storage_infra_get(i, &entry);
            if (entry.device_id != dst_id) {
                next_hop_id[next_hop_count++] = entry.device_id;
            }
            if (entry.device_type == DEVICE_TYPE_GATEWAY) {
                gateway_id = entry.device_id;
            }
        }
    } else if (route != NULL) {
        route_entry_t known_entry = {0};
        uint8_t idx = 0;
        /* The RAM table only mirrors the *best* next-hop. To check whether
         * dst_id is any known next-hop, scan the EEPROM route_info list. */
        for (int i = 0; i < known_route_count; i++) {
            if (known_route_table[i].device_id != route->device_id) {
                continue;
            }
            if (storage_route_get(i, &known_entry) != 0) {
                break;
            }
            for (idx = 0; idx < known_entry.route_count; idx++) {
                if (known_entry.route_info[idx].next_hop_id == dst_id) {
                    break;
                }
            }
            break;
        }

        for (int i = 0; i < storage_infra_count(); i++) {
            storage_infra_get(i, &entry);
            if (entry.device_id != dst_id && entry.device_id != route->device_id && known_entry.route_info[idx].route_length < route->route_len) {
                next_hop_id[next_hop_count++] = entry.device_id;
            }
            if (entry.device_type == DEVICE_TYPE_GATEWAY) {
                gateway_id = entry.device_id;
            }
        }
    } else {
        LOG_ERR("The route info is null when building route info packet for device %d", dst_id);
        return;
    }

    route_info_t forward_pkt = {
        .device_id = is_called_from_pair ? dst_id : route->device_id,
        .device_type = is_called_from_pair ? dst_type : route->device_type,
        .route_len = is_called_from_pair ? 1 : (route->route_len + 1),
        .avg_rssi_2 = is_called_from_pair ? rssi_2 : ( route->avg_rssi_2 + rssi_2) / 2,
    };

    for (int i = 0; i < next_hop_count; i++) {
        send_route_info(&forward_pkt, next_hop_id[i], (gateway_id == next_hop_id[i]) ? DEVICE_TYPE_GATEWAY : DEVICE_TYPE_ANCHOR, (route == NULL) ? STATUS_DEVICE_JOINED : route->hdr.status);
    }

    return;
}

/* Handle received pairing request packet. */
void handle_pair_request(const pair_request_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Sensor's will not process pair request
    if (DEVICE_TYPE == DEVICE_TYPE_SENSOR) {
        return;
    }

    LOG_INF_MAG("   Recieved PAIR_REQUEST from %s ID:%d and RSSI:%d (status: %d)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate sender type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in PAIR_REQUEST from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    /* Verify hash: hash should equal compute_pair_hash(sender_id, random_num). */
    uint32_t expected_hash = compute_pair_hash(dst_id, pkt->random_num);

    if (pkt->hash != expected_hash) {
        LOG_WRN("PAIR_REQUEST hash mismatch from %d (got 0x%08x, expected 0x%08x)", dst_id, pkt->hash, expected_hash);
        send_pair_response(dst_id, pkt->hdr.device_type, pkt->hdr.tracking_id, STATUS_AUTH_FAILED);
        return;
    }

    // For Testing Purpose: Sensor will not pair with gateway
    if (pkt->hdr.device_type == DEVICE_TYPE_SENSOR && DEVICE_TYPE == DEVICE_TYPE_GATEWAY) {
        return;
    }

    uint8_t status;

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if device is already paired with this anchor
                status = check_infra_storage(dst_id, pkt->hdr.device_type, true);
            } else if (pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Check if device is already paired with this sensor
                status = check_sensor_storage(dst_id);
            } else {
                // Reject pair request except from anchor and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            // Sensor can't accept pair request only sends pair request, so reject any incoming pair request
            return;
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any pair request if this device has invalid type
            return;
        }
        break;
    }

    if (status == STATUS_SUCCESS && MESH_DEVICES_COUNT >= MAX_DEVICES) {
        status = STATUS_STORAGE_FULL;
    }

    // Send response based on device type and storage check result
    send_pair_response(dst_id, pkt->hdr.device_type, pkt->hdr.tracking_id, status);

    return;
}

/* Handle received pairing response packet.
 * Collects candidates for 3 seconds, then mesh_tick() selects the best ones. */
void handle_pair_response(const pair_response_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Recieved PAIR_RESPONSE from %s ID:%d and RSSI:%d (status: %d)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in PAIR_RESPONSE from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    if (DEVICE_HOP_NUMBER == 0xFF && pkt->hop_num == 0xFF) {
        // This means the responder is not in the network yet, so reject
        return;
    }

    /* Remove tracker entry for the pair request. */
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

    // Process only packets with status_success or status_already_exixts
    if (pkt->hdr.status != STATUS_SUCCESS && pkt->hdr.status != STATUS_ALREADY_EXISTS) {
        return;
    }

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway can't accept pair response only sends pair response, so reject any incoming pair response
            return;
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if device is already paired with this gateway/anchor
                if (check_infra_storage(dst_id, pkt->hdr.device_type, true) == STATUS_ALREADY_EXISTS) {
                    // send repair request
                    send_repair_request(dst_id, pkt->hdr.device_type);
                    return;
                }
            } else if (pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Check if device is already paired with this sensor
                if (check_sensor_storage(dst_id) == STATUS_ALREADY_EXISTS) {
                    // send repair request
                    send_repair_request(dst_id, pkt->hdr.device_type);
                    return;
                }
            } else {
                // Reject pair response except from anchor, gateway and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if device is already paired with this gateway/anchor
                if (check_infra_storage(dst_id, pkt->hdr.device_type, true) == STATUS_ALREADY_EXISTS) {
                    // send repair request
                    send_repair_request(dst_id, pkt->hdr.device_type);
                    return;
                }
            } else {
                // Reject pair response except from anchor and gateway
                return;
            }
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any pair response if this device has invalid type
            return;
        }
        break;
    }

    /* Start collection window on first response. */
    if (!collecting) {
        collecting = true;
        resp_candidate_count = 0;
        nbtimeout_init(&collect_timer, COLLECT_WINDOW_MS, 0);
        nbtimeout_start(&collect_timer);
        LOG_INF("Collection started - gathering responses for %d ms", COLLECT_WINDOW_MS);
    }

    /* Add to candidates if there's room. */
    if (resp_candidate_count < MAX_RESPONSE_CANDIDATES) {
        resp_candidates[resp_candidate_count++] = (struct response_candidate){
            .sender_id = dst_id,
            .device_type = pkt->hdr.device_type,
            .hop_num = pkt->hop_num,
            .rssi_2 = rssi_2,
        };
        LOG_INF("Candidate %d: %s ID:%d hop:%d RSSI:%d.%d", resp_candidate_count, device_type_str(pkt->hdr.device_type), dst_id, pkt->hop_num, (rssi_2 / 2), (rssi_2 & 0b1) * 5);
    } else {
        LOG_WRN("Candidate buffer full, ignoring response from %d", dst_id);
    }

    return;
}

/* Handle received pairing confirm packet. */
void handle_pair_confirm(const pair_confirm_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Recieved PAIR_CONFIRM from %s ID:%d and RSSI:%d (status: %d)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in PAIR_CONFIRM from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    // Process only packets with status_success
    if (pkt->hdr.status != STATUS_SUCCESS) {
        return;
    }

    uint8_t status;

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if device is already paired with this anchor
                status = check_infra_storage(dst_id, pkt->hdr.device_type, true);
                if (status == STATUS_SUCCESS) {
                    // Add to infra storage
                    infra_entry_t entry;
                    entry.device_id = dst_id;
                    entry.device_type = pkt->hdr.device_type;
                    entry.hop_num = pkt->hop_num == 0xFF ? DEVICE_HOP_NUMBER + 1 : pkt->hop_num;
                    entry.rssi_2 = rssi_2;
                    entry.version = pkt->version;
                    int err = storage_infra_add(&entry);
                    if (err) {
                        LOG_ERR("Failed to store paired anchor, err %d", err);
                        return;
                    }
                    LOG_INF("ANCHOR ID:%d paired and stored in infra (total %d)", dst_id, storage_infra_count());
                } else if (status == STATUS_ALREADY_EXISTS) {
                    LOG_INF("ANCHOR ID:%d already paired, received PAIR_CONFIRM with success", dst_id);
                }
            } else if (pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Check if device is already paired with this sensor
                status = check_sensor_storage(dst_id);
                if (status == STATUS_SUCCESS) {
                    // Add to sensor storage
                    sensor_entry_t entry;
                    entry.device_id = dst_id;
                    entry.version = pkt->version;
                    int err = storage_sensor_add(&entry);
                    if (err) {
                        LOG_ERR("Failed to store paired sensor, err %d", err);
                        return;
                    }
                    LOG_INF("SENSOR ID:%d paired and stored (total %d)", dst_id, storage_sensor_count());
                } else if (status == STATUS_ALREADY_EXISTS) {
                    LOG_INF("SENSOR ID:%d already paired, received PAIR_CONFIRM with success", dst_id);
                }
            } else {
                // Reject pair confirm except from anchor and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            // Sensor can't accept pair confirm only sends pair confirm, so reject any incoming pair confirm
            return;
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any pair confirm if this device has invalid type
            return;
        }
        break;
    }

    send_pair_ack(dst_id, pkt->hdr.device_type, pkt->hdr.tracking_id, status);

    return;

}

/* Handle received pairing acknowledgment packet. */
void handle_pair_ack(const pair_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Recieved PAIR_ACK from %s ID:%d and RSSI:%d (status: %d)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in PAIR_ACK from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    /* Remove tracker entry for the pair request. */
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

    // Process only packets with status_success or status_already_exixts
    if (pkt->hdr.status != STATUS_SUCCESS && pkt->hdr.status != STATUS_ALREADY_EXISTS) {
        return;
    }

    uint8_t status;

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway can't accept pair ack only sends pair ack, so reject any incoming pair ack
            return;
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if device is already paired with this gateway/anchor
                status = check_infra_storage(dst_id, pkt->hdr.device_type, true);
            } else if (pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Check if device is already paired with this sensor
                status = check_sensor_storage(dst_id);
            } else {
                // Reject pair ack except from anchor, gateway and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if device is already paired with this gateway/anchor
                status = check_infra_storage(dst_id, pkt->hdr.device_type, true);
            } else {
                // Reject pair ack except from anchor and gateway
                return;
            }
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any pair confirm if this device has invalid type
            return;
        }
    }

    // If status is success, it means the device is newly paired, add to storage if not exist.
    if (status == STATUS_SUCCESS) {
        infra_entry_t entry;
        entry.device_id = dst_id;
        entry.device_type = pkt->hdr.device_type;
        entry.hop_num = pkt->hop_num;
        entry.rssi_2 = rssi_2;
        entry.version = pkt->version;
        int err = storage_infra_add(&entry);
        if (err) {
            LOG_ERR("Failed to store paired device, err %d", err);
            return;
        }
        LOG_INF("%s ID:%d paired and stored in infra (total %d)", device_type_str(pkt->hdr.device_type), dst_id, storage_infra_count());

        // Send Joined Network packet to the newly paired device
        joined_network_t jn_pkt = {
            .device_type = DEVICE_TYPE,
            .device_id = radio_get_device_id(),
            .serial_num = SERIAL_NUMBER,
            .version = FIRMWARE_VERSION,
        };

        if (DEVICE_TYPE == DEVICE_TYPE_ANCHOR) {
            device_info_update();
            jn_pkt.connected_device_id = 0xFFFF; // Anchor doesn't have connected device ID at this point, set to 0xFFFF to indicate
            jn_pkt.hop_num = DEVICE_HOP_NUMBER;
            jn_pkt.sensor_count = storage_sensor_count();
        } else if (DEVICE_TYPE == DEVICE_TYPE_SENSOR) {
            CONNECTED_DEVICE_ID = dst_id;
            jn_pkt.connected_device_id = dst_id;
            jn_pkt.hop_num = 0xFF;
            jn_pkt.sensor_count = 0xFF;
        }

        if (storage_infra_count() == 1) {
            send_joined_network(&jn_pkt, dst_id, pkt->hdr.device_type, STATUS_SUCCESS);
        }

    } else if (status == STATUS_ALREADY_EXISTS) {
        LOG_INF("%s ID:%d already paired, received PAIR_ACK with success", device_type_str(pkt->hdr.device_type), dst_id);
    } else if (status == STATUS_STORAGE_FULL) {
        LOG_WRN("Storage full, cannot pair with %s ID:%d, received PAIR_ACK with failure", device_type_str(pkt->hdr.device_type), dst_id);
    }
    
    return;
}

/* Handle joined network packet */
void handle_joined_network(const joined_network_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received JOINED_NETWORK from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, device_type_str(pkt->device_type), pkt->device_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate sender type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in JOINED_NETWORK from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    // Process only packets with status_success
    if (pkt->hdr.status != STATUS_SUCCESS) {
        return;
    }

    uint8_t status;

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Check if device is already in the network with this anchor/sensor
                status = check_mesh_storage(pkt->device_id);
                if (status == STATUS_SUCCESS) {
                    // Add to mesh storage
                    mesh_entry_t entry;
                    entry.device_type = pkt->device_type;
                    entry.device_id = pkt->device_id;
                    entry.serial_num = pkt->serial_num;
                    entry.version = pkt->version;
                    entry.connected_device_id = pkt->connected_device_id;
                    entry.hop_num = pkt->hop_num;
                    entry.sensor_count = pkt->sensor_count;
                    int err = storage_mesh_add(&entry);
                    if (err) {
                        LOG_ERR("Failed to store joined mesh device, err %d", err);
                    }
                    LOG_INF("%s ID:%d successfully joined network", device_type_str(pkt->device_type), pkt->device_id);
                } else if (status == STATUS_ALREADY_EXISTS) {
                    LOG_INF("%s ID:%d already in network, received JOINED_NETWORK with success", device_type_str(pkt->device_type), pkt->device_id);
                }
                // Send Ping Device
                ping_known_devices(pkt->device_id, status);
            } else {
                // Reject joined network except from anchor and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Upstream the Packet to Gateway if Anchor receives the JOINED_NETWORK packet from Sensor/Anchor
                joined_network_ack_t ack_pkt = {
                    .dst_device_id = pkt->device_id,
                    .dst_device_type = pkt->device_type,
                };
                send_joined_network_ack(&ack_pkt, dst_id, pkt->hdr.device_type, pkt->hdr.tracking_id, STATUS_SUCCESS);
                LOG_INF("Forwarding JOINED_NETWORK from %s ID:%d  for %s ID:%d upstream", device_type_str(pkt->hdr.device_type), dst_id, device_type_str(pkt->device_type), pkt->device_id);
                infra_entry_t entry;
                storage_infra_get(0, &entry);
                send_joined_network(pkt, entry.device_id, entry.device_type, STATUS_SUCCESS);
                return;
            } else {
                // Reject joined network except from anchor and sensor
                return;
            } 
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            // Sensor can't accept joined network only sends joined network, so reject any incoming joined network
            return;
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any joined network if this device has invalid type
            return;
        }
        break;
    }

    joined_network_ack_t ack_pkt = {
        .dst_device_id = pkt->device_id,
        .dst_device_type = pkt->device_type,
    };
    send_joined_network_ack(&ack_pkt, dst_id, pkt->hdr.device_type, pkt->hdr.tracking_id, status);

    return;
}

/* Handle joined network acknowledgment packet */
void handle_joined_network_ack(const joined_network_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{

    // Only Process if it's for this device
    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received JOINED_NETWORK_ACK from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, device_type_str(pkt->dst_device_type), pkt->dst_device_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in JOINED_NETWORK_ACK from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    /* Remove the joined network tracker. */
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway can't accept joined network ack only sends joined network ack, so reject any incoming joined network ack
            return;
        }
        break;
        
        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Implement Later
            }
            else {
                // Reject joined network ack except from anchor and gateway
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                if (pkt->hdr.status != STATUS_SUCCESS) {
                    // Resend joined network packet. 
                    // Implement later
                    return;
                }
            } else {
                // Reject joined network ack except from anchor and gateway
                return;
            }
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any joined network ack if this device has invalid type
            return;
        }
        break;
    }

    return;
}

/* Handle ping device packet */
void handle_ping_device(const ping_device_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received PING_DEVICE from %s ID:%d with RSSI:%d (status: 0x%02x) (gen_id: %d)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status, pkt->dst_device_id);

    // Validate sender type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in PING_DEVICE from %d, ignoring", pkt->hdr.device_type, dst_id);
        return;
    }

    if (pkt->dst_device_id != 0 || pkt->dst_device_id != 0xFFFF) {
        for (int i = 0; i < infra_count; i++) {
            if (infra_devices[i].entry.device_id == pkt->dst_device_id && pkt->hdr.status != STATUS_ALREADY_EXISTS && pkt->hdr.status != STATUS_SUCCESS) {
                // Remove from Infra Storage
                storage_infra_remove(i);
            }
        }
    }

    if (pkt->dst_device_id == radio_get_device_id()) {
        if (pkt->hdr.status == STATUS_SUCCESS || pkt->hdr.status == STATUS_ALREADY_EXISTS) {
            process_next_request = true;
            // Send Route Info
            // Implement Later if device != Sensor
        } else {
            // factory reset
            factory_reset();
        }
        
    }

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway never receives ping device because it's the root, but in case it receives ping device just ignore
            return;
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY) {
                // Check if we need to update hop number
                if (update_infra_storage(dst_id, pkt->hop_num, rssi_2)) {
                    LOG_INF("Updated hop number for GATEWAY ID:%d to hop:%d based on PING_DEVICE", dst_id, DEVICE_HOP_NUMBER);
                    device_updated_t du_pkt = {
                        .device_type = DEVICE_TYPE,
                        .device_id = radio_get_device_id(),
                        .serial_num = SERIAL_NUMBER,
                        .version = FIRMWARE_VERSION,
                        .connected_device_id = 0xFFFF,
                        .hop_num = DEVICE_HOP_NUMBER,
                        .sensor_count = storage_sensor_count(),
                    };
                    send_device_updated(&du_pkt, dst_id, pkt->hdr.device_type, STATUS_SUCCESS);
                }
            } else if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if we need to update hop number
                if (update_infra_storage(dst_id, pkt->hop_num, rssi_2)) {
                    LOG_INF("Updated hop number for ANCHOR ID:%d to hop:%d based on PING_DEVICE", dst_id, DEVICE_HOP_NUMBER);
                    device_updated_t du_pkt = {
                        .device_type = DEVICE_TYPE,
                        .device_id = radio_get_device_id(),
                        .serial_num = SERIAL_NUMBER,
                        .version = FIRMWARE_VERSION,
                        .connected_device_id = 0xFFFF,
                        .hop_num = DEVICE_HOP_NUMBER,
                        .sensor_count = storage_sensor_count(),
                    };
                    send_device_updated(&du_pkt, dst_id, pkt->hdr.device_type, STATUS_SUCCESS);
                }
            } else {
                // Reject ping device except from gateway, anchor, and sensor
                return;
            }
            // Set the mesh time
            mesh_time_set(pkt->timestamp);
            MESH_DEVICES_COUNT = pkt->total_devices;
            ping_known_devices(pkt->dst_device_id, pkt->hdr.status);
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Update mesh time
                mesh_time_set(pkt->timestamp);
                update_infra_storage(dst_id, pkt->hop_num, rssi_2);
            } else {
                // Reject ping device except from gateway and anchor
                return;
            }
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any ping device if this device has invalid type
            return;
        }
        break;
    }
    // Send ACK back to the sender
    send_ping_ack(dst_id, pkt->hdr.device_type, pkt->dst_device_id, pkt->hdr.tracking_id, STATUS_SUCCESS);

    LOG_INF("Mesh Time %llu seconds", mesh_time_get() / 1000);
    return;
}

/* Handle ping ack packet */
void handle_ping_ack(const ping_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received PING_ACK from %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in PING_ACK from %d, ignoring", pkt->hdr.device_type, dst_id);
        return;
    }

    // Remove the ping tracker
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Update Infra storage with new RSSI and hop number
                update_infra_storage(dst_id, pkt->hop_num, rssi_2);
            } else if (pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Update Sensor storage
                update_sensor_storage(dst_id, pkt->version);
            } else {
                // Reject ping ack except from anchor and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if we need to update hop number
                if (update_infra_storage(dst_id, pkt->hop_num, rssi_2)) {
                    LOG_INF("Updated hop number for ANCHOR ID:%d to hop:%d based on PING_ACK", dst_id, DEVICE_HOP_NUMBER);
                    device_updated_t du_pkt = {
                        .device_type = DEVICE_TYPE,
                        .device_id = radio_get_device_id(),
                        .serial_num = SERIAL_NUMBER,
                        .version = FIRMWARE_VERSION,
                        .connected_device_id = 0xFFFF,
                        .hop_num = DEVICE_HOP_NUMBER,
                        .sensor_count = storage_sensor_count(),
                    };
                    send_device_updated(&du_pkt, dst_id, pkt->hdr.device_type, STATUS_SUCCESS);
                }
            } else if (pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Update Sensor storage
                update_sensor_storage(dst_id, pkt->version);
            } else {
                // Reject ping ack except from gateway, anchor, and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            // Sensor will never receive ping ack, if it receives jut ignore it
            return;
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any ping ack if this device has invalid type
            return;
        }
        break;
    }

    LOG_INF("Mesh Time %llu seconds", mesh_time_get() / 1000);
    return;
}

/* Handle device updated packet */
void handle_device_updated(const device_updated_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received DEVICE_UPDATED from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, device_type_str(pkt->hdr.device_id), pkt->hdr.device_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate sender type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in DEVICE_UPDATED from %d, ignoring", pkt->hdr.device_type, dst_id);
        return;
    }

    // Process only packets with status_success
    if (pkt->hdr.status != STATUS_SUCCESS) {
        return;
    }

    uint8_t status;

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                status = check_mesh_storage(pkt->hdr.device_id);
                if (status == STATUS_ALREADY_EXISTS) {
                    update_mesh_storage(pkt->hdr.device_id, pkt->hop_num, pkt->version, pkt->connected_device_id, pkt->sensor_count);
                    LOG_INF("Updated mesh storage for device %s ID:%d based on DEVICE_UPDATED", device_type_str(pkt->hdr.device_type), pkt->hdr.device_id);
                } else {
                    LOG_WRN("Received DEVICE_UPDATED for device %s ID:%d which is not in mesh storage", device_type_str(pkt->hdr.device_type), pkt->hdr.device_id);
                    status = STATUS_NOT_FOUND;
                }
            } else{
                // Reject device updated except from anchor and sensor
                return;
            }
        }
        break;
        
        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Upstream the update to gateway
                status = STATUS_SUCCESS;
                LOG_INF("Forwarding DEVICE_UPDATED from %s ID:%d, updating infra storage", device_type_str(pkt->hdr.device_type), pkt->hdr.device_id);
                infra_entry_t entry;
                storage_infra_get(0, &entry);
                send_device_updated(pkt, entry.device_id, pkt->hdr.device_type, pkt->hdr.status);
            } else {
                // Reject device updated except from anchor and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            // Sensor will never receive device updated because only anchor and gateway can send device updated, so just ignore if received
            return;
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any device updated if this device has invalid type
            return;
        }
        break;
    }

    device_updated_ack_t ack_pkt = {
        .dst_device_id = pkt->hdr.device_id,
        .dst_device_type = pkt->hdr.device_type,
    };

    send_device_updated_ack(&ack_pkt, dst_id, pkt->hdr.device_type, pkt->hdr.tracking_id, status);

    return;
}

/* Handle device updated acknowledgment packet */
void handle_device_updated_ack(const device_updated_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received DEVICE_UPDATED_ACK from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, device_type_str(pkt->dst_device_type), pkt->dst_device_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in DEVICE_UPDATED_ACK from %d, ignoring", pkt->hdr.device_type, dst_id);
        return;
    }

    // Remove the ping tracker
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway will never receive device updated ack because only anchor can send device updated ack, so just ignore if received
            return;
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
            // Implement any necessary action if needed when anchor receives device updated ack from gateway/anchor
            }
            else {
                // Reject device updated ack except from anchor and gateway
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Implement any necessary action if needed when sensor receives device updated ack from gateway/anchor
            }
            else {
                // Reject device updated ack except from anchor and gateway
                return;
            }
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any device updated ack if this device has invalid type
            return;
        }
        break;
    }

    return;
}

/* Handle repair request packet */
void handle_repair_request(const repair_request_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received REPAIR_REQUEST from %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate sender type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in REPAIR_REQUEST from %d, ignoring", pkt->hdr.device_type, dst_id);
        return;
    }

    /* Verify hash: hash should equal compute_pair_hash(dst_id, req->random_num) */
    uint32_t expected_hash = compute_pair_hash(dst_id, pkt->random_num);

    if (pkt->hash != expected_hash) {
        LOG_WRN("REPAIR_REQUEST hash mismatch from %s ID:%d (got 0x%08x, expected 0x%08x)", device_type_str(pkt->hdr.device_type), dst_id, pkt->hash, expected_hash);
        send_repair_response(dst_id, pkt->hdr.device_type, pkt->hdr.tracking_id, STATUS_AUTH_FAILED);
        return;
    }

    uint8_t status;

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if device is already paired with this anchor
                status = check_infra_storage(dst_id, pkt->hdr.device_type, true);
                status = (status == STATUS_ALREADY_EXISTS) ? STATUS_SUCCESS : STATUS_NOT_FOUND;
            } else if (pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Check if device is already paired with this sensor
                status = check_sensor_storage(dst_id);
                status = (status == STATUS_ALREADY_EXISTS) ? STATUS_SUCCESS : STATUS_NOT_FOUND;
            } else {
                // Reject repair request except from anchor and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            // Sensor can only receive repair request from gateway/anchor, reject if it's from other sensor
            return;
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any repair request if this device has invalid type
            return;
        }
        break;
    }

    send_repair_response(dst_id, pkt->hdr.device_type, pkt->hdr.tracking_id, status);

    return;
}

/* Handle repair response packet */
void handle_repair_response(const repair_response_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received REPAIR_RESPONSE from %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in REPAIR_RESPONSE from %d, ignoring", pkt->hdr.device_type, dst_id);
        return;
    }

    /* Remove the tracker. */
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

    // Process only packets with status_success
    if (pkt->hdr.status != STATUS_SUCCESS) {
        return;
    }

    uint8_t status;

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway never receive repair response because it sends repair response ,so reject incoming repair response
            return;
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_GATEWAY) {
                // Check if device is already paired with this anchor/gateway
                status = check_infra_storage(dst_id, pkt->hdr.device_type, true);
                if (status == STATUS_ALREADY_EXISTS) {
                    // Update the infra storage with new hop number and rssi
                    if (update_infra_storage(dst_id, pkt->hop_num, rssi_2)) {
                        LOG_INF("Updated infra storage for device %s ID:%d based on REPAIR_RESPONSE", device_type_str(pkt->hdr.device_type), dst_id);
                    }
                } else if (status == STATUS_SUCCESS) {
                    infra_entry_t entry;
                    entry.device_id = dst_id;
                    entry.device_type = pkt->hdr.device_type;
                    entry.hop_num = pkt->hop_num;
                    entry.rssi_2 = rssi_2;
                    entry.version = pkt->version;
                    int err = storage_infra_add(&entry);
                    if (err) {
                        LOG_ERR("Failed to store repaired device %s ID:%d, err %d", device_type_str(pkt->hdr.device_type), dst_id, err);
                        return;
                    }
                    LOG_INF("Stored repaired device %s ID:%d successfully", device_type_str(pkt->hdr.device_type), dst_id);
                    device_info_update();
                } else {
                    LOG_WRN("Storage is full, cannot store repaired device %s ID:%d", device_type_str(pkt->hdr.device_type), dst_id);
                    return;
                }
            } else {
                // Reject repair response except from anchor and gateway
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_GATEWAY) {
                // Check if device is already paired with this anchor/gateway
                status = check_infra_storage(pkt->hdr.device_id, pkt->hdr.device_type, false);
                if (status == STATUS_ALREADY_EXISTS) {
                    // Update the infra storage with new hop number and rssi
                    if (update_infra_storage(pkt->hdr.device_id, pkt->hop_num, rssi_2)) {
                        LOG_INF("Updated infra storage for device %s ID:%d based on REPAIR_RESPONSE", device_type_str(pkt->hdr.device_type), pkt->hdr.device_id);
                    }
                } else if (status == STATUS_SUCCESS) {
                    infra_entry_t entry;
                    entry.device_id = dst_id;
                    entry.device_type = pkt->hdr.device_type;
                    entry.hop_num = pkt->hop_num;
                    entry.rssi_2 = rssi_2;
                    entry.version = pkt->version;
                    int err = storage_infra_add(&entry);
                    if (err) {
                        LOG_ERR("Failed to store repaired device %s ID:%d, err %d", device_type_str(pkt->hdr.device_type), dst_id, err);
                        return;
                    }
                    LOG_INF("Stored repaired device %s ID:%d successfully", device_type_str(pkt->hdr.device_type), dst_id);
                    device_info_update();
                } else {
                    LOG_WRN("Storage is full, cannot store repaired device %s ID:%d", device_type_str(pkt->hdr.device_type), dst_id);
                    return;
                }
            } else {
                // Reject repair response except from anchor and gateway
                return;
            }
        }
        break;

        default:
        {
            // There are only 3 valid Device types, reject any repair response if this device has invlaid type
            return;
        }
        break;
    }
    
    return;
}

/* Handle route info packet */
void handle_route_info(const route_info_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const route_info_t *route = (const route_info_t *)pkt;

    if (route->hdr.device_id != radio_get_device_id()) {
        return;
    }

    int err = 0;
    uint8_t status;

    LOG_INF_MAG("   Received ROUTE_INFO from %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(route->hdr.device_type), dst_id, (rssi_2 / 2), route->hdr.status);

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            if (route->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Update the route info based on status
                if (route->hdr.status == STATUS_DEVICE_JOINED) {
                    err = storage_route_add(dst_id, route->device_id, route->device_type, route->route_len, (route->avg_rssi_2 + rssi_2) / 2);
                    if (err) {
                        LOG_ERR("Failed to add route for device %s ID:%d, err %d", device_type_str(route->hdr.device_type), route->device_id, err);
                        status = STATUS_FAILURE;
                    }
                    LOG_INF("Added route for device %s ID:%d via %d", device_type_str(route->hdr.device_type), route->device_id, dst_id);
                    status = STATUS_SUCCESS;
                } else if (route->hdr.status == STATUS_DEVICE_REMOVED) {
                    err = storage_route_remove(dst_id, route->device_id);
                    if (err) {
                        LOG_ERR("Failed to remove route for device %s ID:%d, err %d", device_type_str(route->hdr.device_type), route->device_id, err);
                        status = STATUS_FAILURE;
                    }
                    LOG_INF("Removed route for device %s ID:%d via %d", device_type_str(route->hdr.device_type), route->device_id, dst_id);
                    status = STATUS_SUCCESS;
                }

                // Pass the route info to upstream devices if the route is updated successfully
                if (status == STATUS_SUCCESS) {
                    build_route_info_and_send(dst_id, route->hdr.device_type, route, rssi_2, false);
                }
            }
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (route->hdr.device_type == DEVICE_TYPE_ANCHOR || route->hdr.device_type == DEVICE_TYPE_GATEWAY) {
                // Update the route info based on status
                if (route->hdr.status == STATUS_DEVICE_JOINED) {
                    err = storage_route_add(dst_id, route->device_id, route->device_type, route->route_len, (route->avg_rssi_2 + rssi_2) / 2);
                    if (err) {
                        LOG_ERR("Failed to add route for device %s ID:%d, err %d", device_type_str(route->hdr.device_type), route->device_id, err);
                        status = STATUS_FAILURE;
                    }
                    LOG_INF("Added route for device %s ID:%d via %d", device_type_str(route->hdr.device_type), route->device_id, dst_id);
                    status = STATUS_SUCCESS;
                } else if (route->hdr.status == STATUS_DEVICE_REMOVED) {
                    err = storage_route_remove(dst_id, route->device_id);
                    if (err) {
                        LOG_ERR("Failed to remove route for device %s ID:%d, err %d", device_type_str(route->hdr.device_type), route->device_id, err);
                        status = STATUS_FAILURE;
                    }
                    LOG_INF("Removed route for device %s ID:%d via %d", device_type_str(route->hdr.device_type), route->device_id, dst_id);
                    status = STATUS_SUCCESS;
                }

                // Pass the route info to upstream devices if the route is updated successfully
                if (status == STATUS_SUCCESS) {
                    build_route_info_and_send(dst_id, route->hdr.device_type, route, rssi_2, false);
                }
            } 
        }
        break;

        case DEVICE_TYPE_SENSOR:
        return;

        default:
            LOG_WRN("Unknown device type 0x%02x in ROUTE_INFO from %d, ignoring", route->hdr.device_type, dst_id);
            return;
    }
    route_info_ack_t ack_pkt = {
        .device_id = route->device_id,
        .device_type = route->device_type,
        .route_len = route->route_len,
        .avg_rssi_2 = route->avg_rssi_2,
    };
    send_route_info_ack(&ack_pkt, dst_id, route->hdr.device_type, route->hdr.tracking_id, status);
    return;
}

/* Handle route info acknowledgment packet */
void handle_route_info_ack(const route_info_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const route_info_ack_t *ack = (const route_info_ack_t *)pkt;

    if (ack->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received ROUTE_INFO_ACK from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(ack->hdr.device_type), dst_id, device_type_str(ack->device_type), ack->device_id, (rssi_2 / 2), ack->hdr.status);

    /* Remove the tracker. */
    tracker_remove_by_tracking_id(ack->hdr.tracking_id);

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
            return;

        case DEVICE_TYPE_ANCHOR:
            if (ack->hdr.device_type == DEVICE_TYPE_ANCHOR || ack->hdr.device_type == DEVICE_TYPE_GATEWAY) {
                if (ack->hdr.status != STATUS_SUCCESS) {
                    // Resend to that device
                    route_info_t pkt = {
                        .device_id = ack->device_id,
                        .device_type = ack->device_type,
                        .route_len = ack->route_len,
                        .avg_rssi_2 = ack->avg_rssi_2,
                    };
                    LOG_WRN("Received failure ROUTE_INFO_ACK for device %s ID:%d from %d, resending ROUTE_INFO", device_type_str(ack->hdr.device_type), ack->device_id, dst_id);
                    send_route_info(&pkt, dst_id, ack->hdr.device_type, ack->hdr.status);
                }
            }
            return;

        case DEVICE_TYPE_SENSOR:
            return;

        default:
            LOG_WRN("Unknown device type 0x%02x in ROUTE_INFO_ACK from %d, ignoring", ack->hdr.device_type, dst_id);
            return;
    }
    return;
}

void mesh_tick(void)
{
    if (process_next_request && resp_candidate_idx > 0) {
        process_next_request = false;
        select_and_confirm(resp_candidate_count - resp_candidate_idx);
        resp_candidate_idx--;
    }

    if (!collecting) {
        return;
    }

    if (nbtimeout_expired(&collect_timer)) {
        LOG_INF("Collection window expired - %d candidates", resp_candidate_count);
        collecting = false;
        nbtimeout_stop(&collect_timer);
        sort_candidates();

        int available = MAX_ANCHORS - storage_infra_count();
        LOG_INF("Available slots for anchors: %d", available);
        if (available <= 0) {
            LOG_WRN("No available slots for anchors, cannot process any more candidates");
            resp_candidate_count = 0;
        } else {
            resp_candidate_count = (resp_candidate_count < available) ? resp_candidate_count : available;
            process_next_request = true;
        }
        resp_candidate_idx = resp_candidate_count;
    }

}

/* Compute hash. */
uint32_t compute_pair_hash(uint16_t dev_id, uint32_t random_num)
{
    uint32_t hash = (uint32_t)dev_id ^ random_num;
    hash = ((hash << 13) | (hash >> 19)) ^ (hash * 0x02152001);
    return hash;
}