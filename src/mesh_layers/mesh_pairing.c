

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include "mesh_pairing.h"
#include "mesh_session.h"
#include "mesh_routing.h"
#include "../mesh.h"
#include "../protocol.h"
#include "../tracker.h"
#include "../queue.h"
#include "../product_info.h"
#include "../log_color.h"
#include "../radio.h"
#include "../storage.h"

LOG_MODULE_REGISTER(mesh_pairing, CONFIG_MESH_PAIRING_LOG_LEVEL);

/* Candidate from a PAIR_RESPONSE during collection. */
struct response_candidate {
    uint16_t sender_id;
    uint8_t  device_type;
    uint8_t  hop_num;
    int16_t  rssi_2;
};

static struct response_candidate resp_candidates[MAX_PAIR_RESPONSE_CANDIDATES];
static int resp_candidate_count;
static int resp_candidate_idx = 0;
static struct nbtimeout collect_timer;
static bool collecting = false;
static bool process_next_request = false;

void mesh_pairing_set_pending_request(void)
{
    process_next_request = true;
}

/* Generate random number for pairing. */
static uint32_t generate_random_number(void)
{
    uint32_t random_num;
    sys_rand_get(&random_num, sizeof(random_num));
    return random_num;
}

/* Compute hash. */
static uint32_t compute_pair_hash(uint16_t dev_id, uint32_t random_num)
{
    uint32_t hash = (uint32_t)dev_id ^ random_num;
    hash = ((hash << 13) | (hash >> 19)) ^ (hash * 0x02152001);
    return hash;
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
    uint8_t status = check_infra_storage(c->sender_id, c->device_type, false);

    if (status == STATUS_STORAGE_FULL) {
        LOG_WRN("Infra storage full, stopping selection");
        resp_candidate_count = 0;
        return;
    }

    if (get_hop_number() == 0xFF && c->hop_num == 0xFF) {
        LOG_WRN("Candidate %s ID:%d has no upstream link or has invalid hop number %d, skipping", device_type_str(c->device_type), c->sender_id, c->hop_num);
        process_next_request = true;
        return;
    }

    send_pair_confirm(c->sender_id, c->device_type, STATUS_SUCCESS);

    if (get_device_type() == DEVICE_TYPE_SENSOR) {
        // Sensor only pairs with one anchor, so stop after the first confirmation
        resp_candidate_count = 0;
        return;
    }
}

static void build_joined_network(const pair_ack_t *pkt, uint16_t dst_id, int16_t rssi_2) {
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

    // Update connected device ID and hop number
    device_info_update();

    // Send Joined Network packet to the newly paired device
    joined_network_t jn_pkt = {
        .device_type = get_device_type(),
        .device_id = get_device_id(),
        .serial_num = get_serial_number(),
        .version = get_firmware_version(),
    };

    if (get_device_type() == DEVICE_TYPE_ANCHOR) {
        jn_pkt.connected_device_id = infra_devices[0].entry.device_id;
        jn_pkt.hop_num = get_hop_number();
        jn_pkt.sensor_count = storage_sensor_count();
    } else if (get_device_type() == DEVICE_TYPE_SENSOR) {
        set_connected_device_id(dst_id);
        jn_pkt.connected_device_id = dst_id;
        jn_pkt.hop_num = 0xFF;
        jn_pkt.sensor_count = 0xFF;
    }

    if (storage_infra_count() == 1) {
        send_joined_network(&jn_pkt, dst_id, pkt->hdr.device_type, STATUS_SUCCESS);
    }
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------------**** TX Helpers ****------------------------------------------------------------------------------- */

/* Send pairing request packet. */
int send_pair_request(bool include_tracker)
{
    uint32_t random_num = generate_random_number();
    uint32_t hash = compute_pair_hash(get_device_id(), random_num);

    pair_request_t packet = {
        .hdr = {
            .packet_type = PACKET_PAIR_REQUEST,
            .device_type = get_device_type(),
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracker_next_id(),
            .device_id = 0,
            .status = STATUS_SUCCESS,
        },
        .random_num = random_num,
        .hash = hash,
    };

    // Add tracker entry for retries
    if (include_tracker) {
        tracker_add(get_device_id(), 0, packet.hdr.tracking_id, PACKET_PAIR_REQUEST, 2 * PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));
    }

    LOG_INF_GRN("Sending PAIR_REQUEST");
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send pairing response packet. */
int send_pair_response(uint16_t dst_id,uint8_t dst_type ,uint8_t tracking_id, uint8_t status)
{
    pair_response_t packet = {
        .hdr = {
            .packet_type = PACKET_PAIR_RESPONSE,
            .device_type = get_device_type(),
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .hop_num = get_hop_number(),
    };

    LOG_INF_GRN("Sending PAIR_RESPONSE to device %s ID:%d with hop_num %d (status: 0x%02x)", device_type_str(dst_type), dst_id, get_hop_number(), status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send pairing confirm packet. */
int send_pair_confirm(uint16_t dst_id, uint8_t dst_type, uint8_t status)
{

    pair_confirm_t packet = {
        .hdr = {
            .packet_type = PACKET_PAIR_CONFIRM,
            .device_type = get_device_type(),
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracker_next_id(),
            .device_id = dst_id,
            .status = status,
        },
        .version = get_firmware_version(),
        .hop_num = get_hop_number(),
    };

    // Add tracker entry for retries
    tracker_add(dst_id, get_device_id(), packet.hdr.tracking_id, PACKET_PAIR_CONFIRM, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF_GRN("Sending PAIR_CONFIRM to device %s ID:%d with hop_num %d (status: 0x%02x)", device_type_str(dst_type), dst_id, get_hop_number(), status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send pairing acknowledgment packet. */
int send_pair_ack(uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status)
{
    pair_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_PAIR_ACK,
            .device_type = get_device_type(),
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .hop_num = get_hop_number(),
    };

    LOG_INF_GRN("Sending PAIR_ACK to device %s ID:%d with hop_num %d (status: 0x%02x)", device_type_str(dst_type), dst_id, get_hop_number(), status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send repair request packet. */
int send_repair_request(uint16_t dst_id, uint8_t dst_type)
{
    uint32_t random_num = generate_random_number();
    uint32_t hash = compute_pair_hash(get_device_id(), random_num);

    repair_request_t packet = {
        .hdr = {
            .packet_type = PACKET_REPAIR_REQUEST,
            .device_type = get_device_type(),
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracker_next_id(),
            .device_id = dst_id,
            .status = STATUS_SUCCESS,
        },
        .random_num = random_num,
        .hash = hash,
        .version = get_firmware_version(),
        .hop_num = get_hop_number(),
    };

    // Add tracker entry for retries
    tracker_add(dst_id, get_device_id(), packet.hdr.tracking_id, PACKET_REPAIR_REQUEST, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF_GRN("Sending REPAIR_REQUEST to device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, STATUS_SUCCESS);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send repair response packet. */
int send_repair_response(uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status)
{
    repair_response_t packet = {
        .hdr = {
            .packet_type = PACKET_REPAIR_RESPONSE,
            .device_type = get_device_type(),
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .version = get_firmware_version(),
        .hop_num = get_hop_number(),
    };

    LOG_INF_GRN("Sending REPAIR_RESPONSE to device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------**** Handlers Functions ****--------------------------------------------------------------------------- */

/* Handle received pairing request packet. */
void handle_pair_request(const pair_request_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Sensor's will not process pair request
    if (get_device_type() == DEVICE_TYPE_SENSOR) {
        return;
    }

    // // For Testing Purpose: Sensor will not pair with gateway
    // if ((pkt->hdr.device_type == DEVICE_TYPE_SENSOR || dst_id != 0x926B) && get_device_type() == DEVICE_TYPE_GATEWAY) {
    //     return;
    // }

    uint8_t status;

    if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
        // Check if device is already paired with this anchor
        status = check_infra_storage(dst_id, pkt->hdr.device_type, false);
    } else if (pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
        // Check if device is already paired with this sensor
        status = check_sensor_storage(dst_id);
    } else {
        // Reject pair request except from anchor and sensor
        return;
    }

    if (status == STATUS_ALREADY_EXISTS) {
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

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        case DEVICE_TYPE_ANCHOR:
        {
            if (status == STATUS_SUCCESS && get_mesh_devices_count() >= MAX_DEVICES) {
                status = STATUS_STORAGE_FULL;
            }

            // Send response based on device type and storage check result
            send_pair_response(dst_id, pkt->hdr.device_type, pkt->hdr.tracking_id, status);
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

    return;
}

/* Handle received pairing response packet.
 * Collects candidates for 3 seconds, then mesh_tick() selects the best ones. */
void handle_pair_response(const pair_response_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Recieved PAIR_RESPONSE from %s ID:%d and RSSI:%d (status: %d)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in PAIR_RESPONSE from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    if (get_hop_number() == 0xFF && pkt->hop_num == 0xFF) {
        // This means the responder is not in the network yet, so reject
        return;
    }

    /* Remove tracker entry for the pair request. */
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

    // Process only packets with status_success
    if (pkt->hdr.status != STATUS_SUCCESS) {
        return;
    }

    switch (get_device_type()) {
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
                if (check_infra_storage(dst_id, pkt->hdr.device_type, false) == STATUS_ALREADY_EXISTS) {
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
        nbtimeout_init(&collect_timer, PAIR_RESPONSE_COLLECT_WINDOW_MS, 0);
        nbtimeout_start(&collect_timer);
        LOG_INF("Collection started - gathering responses for %d ms", PAIR_RESPONSE_COLLECT_WINDOW_MS);
    }

    /* Add to candidates if there's room. */
    if (resp_candidate_count < MAX_PAIR_RESPONSE_CANDIDATES) {
        // Check if the candidate is already in the list (can happen if multiple responses are received due to retries)
        for (int i = 0; i < resp_candidate_count; i++) {
            if (resp_candidates[i].sender_id == dst_id) {
                return;
            }
        }
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
    if (pkt->hdr.device_id != get_device_id()) {
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

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if device is already paired with this anchor
                status = check_infra_storage(dst_id, pkt->hdr.device_type, false);
                if (status == STATUS_SUCCESS) {
                    // Add to infra storage
                    infra_entry_t entry;
                    entry.device_id = dst_id;
                    entry.device_type = pkt->hdr.device_type;
                    entry.hop_num = pkt->hop_num == 0xFF ? get_hop_number() + 1 : pkt->hop_num;
                    entry.rssi_2 = rssi_2;
                    entry.version = pkt->version;
                    int err = storage_infra_add(&entry);
                    if (err) {
                        LOG_ERR("Failed to store paired anchor, err %d", err);
                        return;
                    }
                    device_info_update();
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
    if (pkt->hdr.device_id != get_device_id()) {
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

    switch (get_device_type()) {
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
                status = check_infra_storage(dst_id, pkt->hdr.device_type, false);
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
        build_joined_network(pkt, dst_id, rssi_2);
    } else if (status == STATUS_ALREADY_EXISTS) {
        LOG_INF("%s ID:%d already paired, received PAIR_ACK with success", device_type_str(pkt->hdr.device_type), dst_id);
    } else if (status == STATUS_STORAGE_FULL) {
        LOG_WRN("Storage full, cannot pair with %s ID:%d, received PAIR_ACK with failure", device_type_str(pkt->hdr.device_type), dst_id);
    }
    
    return;
}

/* Handle repair request packet */
void handle_repair_request(const repair_request_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
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

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if device is already paired with this anchor
                status = check_infra_storage(dst_id, pkt->hdr.device_type, false);
                if (status == STATUS_SUCCESS && pkt->hop_num != 0xFF) {
                    // Add to infra storage
                    infra_entry_t entry;
                    entry.device_id = dst_id;
                    entry.device_type = pkt->hdr.device_type;
                    entry.hop_num = pkt->hop_num == pkt->hop_num;
                    entry.rssi_2 = rssi_2;
                    entry.version = pkt->version;
                    int err = storage_infra_add(&entry);
                    if (err) {
                        LOG_ERR("Failed to store repaired anchor, err %d", err);
                        return;
                    }
                    device_info_update();
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
                        LOG_ERR("Failed to store repaired sensor, err %d", err);
                        return;
                    }
                }
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
    if (pkt->hdr.device_id != get_device_id()) {
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
    if (pkt->hdr.status != STATUS_SUCCESS && pkt->hdr.status != STATUS_ALREADY_EXISTS && pkt->hdr.status != STATUS_STORAGE_FULL) {
        return;
    }

    uint8_t status;

    switch (get_device_type()) {
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
                if (pkt->hdr.status == STATUS_STORAGE_FULL) {
                    LOG_WRN("Storage full, cannot store repaired device %s ID:%d, removing from storage", device_type_str(pkt->hdr.device_type), dst_id);
                    for (int i = 0; i < infra_count; i++) {
                        if (infra_devices[i].entry.device_id == dst_id) {
                            storage_infra_remove(i);
                            device_info_update();
                            break;
                        }
                        if (i == infra_count - 1) {
                            LOG_WRN("Device %s ID:%d not found in storage when trying to remove after receiving STORAGE_FULL status, maybe already removed", device_type_str(pkt->hdr.device_type), dst_id);
                        }
                    }
                    return;
                }
                status = check_infra_storage(dst_id, pkt->hdr.device_type, false);
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
                status = check_infra_storage(pkt->hdr.device_id, pkt->hdr.device_type, true);
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

void mesh_pairing_tick(void)
{
    if (get_device_type() == DEVICE_TYPE_GATEWAY) {
        // Gateway does not need to collect candidates and select anchors, just return here
        return;
    }
    
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