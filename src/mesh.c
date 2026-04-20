/*
 * Mesh protocol shared utilities for DECT NR+ mesh network
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include "mesh.h"
#include "main_sub.h"
#include "tracker.h"
#include "radio.h"
#include "queue.h"
#include "storage.h"
#include "protocol.h"
#include "product_info.h"

LOG_MODULE_REGISTER(mesh, CONFIG_MESH_LOG_LEVEL);

#include "timeout.h"

#define COLLECT_WINDOW_MS     2000
#define MAX_RESPONSE_CANDIDATES 16

/* Candidate from a PAIR_RESPONSE during collection. */
struct response_candidate {
    uint16_t sender_id;
    uint8_t  device_type;
    uint8_t  hop_num;
    int16_t  rssi_2;
    uint32_t hash;
    uint8_t  tracking_id;
};

static struct response_candidate resp_candidates[MAX_RESPONSE_CANDIDATES];
static int resp_candidate_count;
static struct nbtimeout collect_timer;
static bool collecting;

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

/* Check Infra Storage */
static uint8_t check_infra_storage(uint16_t device_id, uint8_t device_type, bool all_slots)
{
    infra_entry_t entry;

    // Check Only first slot as sensor can only pair with one gateway/anchor, but anchor can pair with multiple sensors
    if (all_slots == false) {
        if (storage_infra_get(0, &entry) == 0 && entry.device_id == device_id) {
            LOG_WRN("%s %d already stored in infra, skipping add", device_type_str(device_type), device_id);
            return STATUS_ALREADY_EXISTS;
        }
        
        if (storage_infra_count() >= 1) {
            LOG_WRN("Infra storage full, cannot add %s %d", device_type_str(device_type), device_id);
            return STATUS_STORAGE_FULL;
        }

        return STATUS_SUCCESS;
    }

    for (int i = 0; i < storage_infra_count(); i++) {
        if (storage_infra_get(i, &entry) == 0 && entry.device_id == device_id) {
            LOG_WRN("%s %d already stored in infra, skipping add", device_type_str(device_type), device_id);
            return STATUS_ALREADY_EXISTS;
        }
    }

    if (storage_infra_count() >= STORAGE_PART1_MAX_ENTRIES) {
        LOG_WRN("Infra storage full, cannot add %s %d", device_type_str(device_type), device_id);
        return STATUS_STORAGE_FULL;
    }

    return STATUS_SUCCESS;
}

/* Check Sensor Storage */
static uint8_t check_sensor_storage(uint16_t device_id)
{
    sensor_entry_t entry;

    for (int i = 0; i < storage_sensor_count(); i++) {
        if (storage_sensor_get(i, &entry) == 0 && entry.device_id == device_id) {
            LOG_WRN("Sensor %d already stored, skipping add", device_id);
            return STATUS_ALREADY_EXISTS;
        }
    }

    if (storage_sensor_count() >= STORAGE_PART2_MAX_ENTRIES) {
        LOG_WRN("Sensor storage full, cannot add sensor %d", device_id);
        return STATUS_STORAGE_FULL;
    }

    return STATUS_SUCCESS;
}

/* Check Mesh Storage */
static uint8_t check_mesh_storage(uint16_t device_id)
{
    mesh_entry_t entry;

    for (int i = 0; i < storage_mesh_count(); i++) {
        if (storage_mesh_get(i, &entry) == 0 && entry.device_id == device_id) {
            LOG_WRN("Mesh %d already stored, skipping add", device_id);
            return STATUS_ALREADY_EXISTS;
        }
    }

    if (storage_mesh_count() >= STORAGE_PART3_MAX_ENTRIES) {
        LOG_WRN("Mesh storage full, cannot add mesh %d", device_id);
        return STATUS_STORAGE_FULL;
    }

    return STATUS_SUCCESS;
}

/* Send pairing request packet. */
int send_pair_request(uint32_t handle, uint8_t tracking_id)
{
    uint32_t random_num = generate_random_number();
    pair_request_t packet = {
        .hdr = {
            .packet_type = PACKET_PAIR_REQUEST,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = 0,
            .status = STATUS_SUCCESS,
        },
        .random_num = random_num,
    };

    return tx_queue_put(&packet, sizeof(packet), QUEUE_PRIO_HIGH);
}

/* Send pairing response packet. */
int send_pair_response(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status, uint32_t hash, uint8_t hop_num)
{
    pair_response_t packet = {
        .hdr = {
            .packet_type = PACKET_PAIR_RESPONSE,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .hash = hash,
        .hop_num = hop_num,
    };

    return tx_queue_put(&packet, sizeof(packet), QUEUE_PRIO_HIGH);
}

/* Send pairing confirm packet. */
int send_pair_confirm(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status)
{
    pair_confirm_t packet = {
        .hdr = {
            .packet_type = PACKET_PAIR_CONFIRM,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
    };

    return tx_queue_put(&packet, sizeof(packet), QUEUE_PRIO_HIGH);
}

/* Send pairing acknowledgment packet. */
int send_pair_ack(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status, uint8_t hop_num)
{
    pair_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_PAIR_ACK,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .hop_num = hop_num,
    };

    return tx_queue_put(&packet, sizeof(packet), QUEUE_PRIO_HIGH);
}

/* Send joined network packet. */
int send_joined_network(uint32_t handle,const joined_network_t *pkt, uint16_t dst_id, uint8_t tracking_id, uint8_t status)
{
    joined_network_t packet = {
        .hdr = {
            .packet_type = PACKET_JOINED_NETWORK,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .device_type = pkt->device_type,
        .device_id = pkt->device_id,
        .serial_num = pkt->serial_num,
        .version = pkt->version,
        .connected_device_id = pkt->connected_device_id,
        .hop_num = pkt->hop_num,
        .sensor_count = pkt->sensor_count,
    };

    return tx_queue_put(&packet, sizeof(packet), QUEUE_PRIO_HIGH);
}

/* Send joined network acknowledgment packet. */
int send_joined_network_ack(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status)
{
    joined_network_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_JOINED_NETWORK_ACK,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
    };

    return tx_queue_put(&packet, sizeof(packet), QUEUE_PRIO_HIGH);
}

/* Handle received pairing request packet. */
void handle_pair_request(const pair_request_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    LOG_INF("PAIR_REQUEST from %s ID:%d and RSSI:%d", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2));

    uint8_t status;

    switch(pkt->hdr.device_type) {
        case DEVICE_TYPE_GATEWAY:
            LOG_WRN("Received PAIR_REQUEST from gateway device %d, ignoring", dst_id);
            return;
        case DEVICE_TYPE_ANCHOR:
            // Check if device is already paired with this anchor
            status = check_infra_storage(dst_id, pkt->hdr.device_type, true);
            break;
        case DEVICE_TYPE_SENSOR:
            // Check if device is already paired with this gateway/anchor
            status = check_sensor_storage(dst_id);
            break;
        default:
            LOG_WRN("Unknown device type 0x%02x in pair request from %d, rejecting", pkt->hdr.device_type, dst_id);
            return;
    }

    uint32_t hash = compute_pair_hash(dst_id, pkt->random_num);

    LOG_INF("Sending PAIR_RESPONSE to %d: status 0x%02x, hash 0x%08x", dst_id, status, hash);
    send_pair_response(0, dst_id, pkt->hdr.tracking_id, status, hash, PRODUCT_HOP_NUMBER);
}

/* Handle received pairing response packet.
 * Collects candidates for 3 seconds, then mesh_tick() selects the best ones. */
void handle_pair_response(const pair_response_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const pair_response_t *resp = (const pair_response_t *)pkt;

    if (resp->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("PAIR_RESPONSE from %s ID:%d: status 0x%02x, hop %d, RSSI %d.%d",
        device_type_str(resp->hdr.device_type), dst_id, resp->hdr.status,
        resp->hop_num, (rssi_2 / 2), (rssi_2 & 0b1) * 5);

    /* Stop the request tracker — we got a response. */
    int idx = tracker_find_by_tracking_id(resp->hdr.tracking_id);
    if (idx >= 0) {
        tracker_remove(idx);
    }

    if (resp->hdr.status != STATUS_SUCCESS) {
        LOG_WRN("PAIR_RESPONSE failed: status 0x%02x", resp->hdr.status);
        return;
    }

    /* Validate responder type. */
    if (resp->hdr.device_type != DEVICE_TYPE_GATEWAY &&
        resp->hdr.device_type != DEVICE_TYPE_ANCHOR) {
        LOG_WRN("PAIR_RESPONSE from unexpected %s, ignoring",
            device_type_str(resp->hdr.device_type));
        return;
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
            .device_type = resp->hdr.device_type,
            .hop_num = resp->hop_num,
            .rssi_2 = rssi_2,
            .hash = resp->hash,
            .tracking_id = resp->hdr.tracking_id,
        };
        LOG_INF("Candidate %d: %s ID:%d hop:%d RSSI:%d.%d",
            resp_candidate_count, device_type_str(resp->hdr.device_type),
            dst_id, resp->hop_num, (rssi_2 / 2), (rssi_2 & 0b1) * 5);
    } else {
        LOG_WRN("Candidate buffer full, ignoring response from %d", dst_id);
    }
}

/* Handle received pairing confirm packet. */
void handle_pair_confirm(const pair_confirm_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const pair_confirm_t *conf = (const pair_confirm_t *)pkt;

    if (conf->hdr.device_id != radio_get_device_id()) {
         return;
    }

    LOG_INF("PAIR_CONFIRM from device %s ID:%d: status 0x%02x", device_type_str(conf->hdr.device_type), dst_id, conf->hdr.status);
    if(((conf->hdr.device_type != DEVICE_TYPE_SENSOR) && (conf->hdr.device_type != DEVICE_TYPE_ANCHOR)) || (conf->hdr.status != STATUS_SUCCESS)) {
        LOG_WRN("PAIR_CONFIRM from unsupported device %s ID:%d: OR status 0x%02x", device_type_str(conf->hdr.device_type), dst_id, conf->hdr.status);
        return;
    }

    uint8_t status;

    switch(pkt->hdr.device_type) {
        case DEVICE_TYPE_GATEWAY:
            LOG_WRN("Received PAIR_CONFIRM from gateway device %d, ignoring", dst_id);
            return;
        case DEVICE_TYPE_ANCHOR:
            // Check if device is already paired with this anchor
            status = check_infra_storage(dst_id, pkt->hdr.device_type, true);
            if (status == STATUS_SUCCESS) {
                infra_entry_t entry;
                entry.device_id = dst_id;
                entry.device_type = conf->hdr.device_type;
                entry.hop_num = 0xFF;
                entry.rssi_2 = rssi_2;
                int err = storage_infra_add(&entry);
                if (err) {
                    LOG_ERR("Failed to store paired anchor, err %d", err);
                    return;
                }
                LOG_INF("Anchor %d paired and stored in infra (total %d)", dst_id, storage_infra_count());
                radio_update_known_devices();
            } else if (status == STATUS_ALREADY_EXISTS) {
                LOG_INF("Anchor %d already paired, received PAIR_CONFIRM with success", dst_id);
            }
            break;
        case DEVICE_TYPE_SENSOR:
            // Check if device is already paired with this gateway/anchor
            status = check_sensor_storage(dst_id);
            if (status == STATUS_SUCCESS) {
                sensor_entry_t entry;
                entry.device_id = dst_id;
                int err = storage_sensor_add(&entry);
                if (err) {
                    LOG_ERR("Failed to store paired sensor, err %d", err);
                    return;
                }
                LOG_INF("Sensor %d paired and stored (total %d)", dst_id, storage_sensor_count());
                radio_update_known_devices();
            } else if (status == STATUS_ALREADY_EXISTS) {
                LOG_INF("Sensor %d already paired, received PAIR_CONFIRM with success", dst_id);
            }
            break;
        default:
            LOG_WRN("Unknown device type 0x%02x in PAIR_CONFIRM from %d, rejecting", pkt->hdr.device_type, dst_id);
            return;
    }

    LOG_INF("Sending PAIR_ACK to %d: status 0x%02x", dst_id, status);
    send_pair_ack(0, dst_id, conf->hdr.tracking_id, status, PRODUCT_HOP_NUMBER);
    return;

}

/* Handle received pairing acknowledgment packet. */
void handle_pair_ack(const pair_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
   const pair_ack_t *ack = (const pair_ack_t *)pkt;

    if (ack->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("PAIR_ACK from %s ID:%d: status 0x%02x", device_type_str(ack->hdr.device_type), dst_id, ack->hdr.status);

    /* Find and remove the confirm tracker by tracking ID from the packet. */
    int idx = tracker_find_by_tracking_id(ack->hdr.tracking_id);
    if (idx >= 0) {
        tracker_remove(idx);
    }

    uint8_t status;

    if (ack->hdr.status == STATUS_SUCCESS || ack->hdr.status == STATUS_ALREADY_EXISTS) {
        switch (ack->hdr.device_type) {
            case DEVICE_TYPE_GATEWAY:
            case DEVICE_TYPE_ANCHOR:
            {
                switch(PRODUCT_DEVICE_TYPE) {
                    case DEVICE_TYPE_GATEWAY:
                        LOG_WRN("Gateway will never receive PAIR_ACK, ignoring");
                        return;
                    
                    case DEVICE_TYPE_ANCHOR:
                        // Check if device is already paired with this anchor
                        status = check_infra_storage(dst_id, pkt->hdr.device_type, true);
                        break;

                    case DEVICE_TYPE_SENSOR:
                        // Check if device is already paired with this gateway/anchor
                        status = check_infra_storage(dst_id, pkt->hdr.device_type, false);
                        break;

                    default:
                        LOG_WRN("Unknown device type 0x%02x in pair ack from %d, ignoring", ack->hdr.device_type, dst_id);
                        return;
                }
                break;
            }
            case DEVICE_TYPE_SENSOR:
                LOG_WRN("Received PAIR_ACK from sensor device %d, ignoring", dst_id);
                return;
            default:
                LOG_WRN("Unknown device type 0x%02x in PAIR_ACK from %d, ignoring", ack->hdr.device_type, dst_id);
                return;
        }
    } else {
        LOG_WRN("PAIR_ACK failed: status 0x%02x", ack->hdr.status);
        return;
    }

    if (status == STATUS_SUCCESS) {
        infra_entry_t entry;
        entry.device_id = dst_id;
        entry.device_type = ack->hdr.device_type;
        entry.hop_num = ack->hop_num;
        entry.rssi_2 = rssi_2;
        int err = storage_infra_add(&entry);
        if (err) {
            LOG_ERR("Failed to store paired device, err %d", err);
            return;
        }
        LOG_INF("%s %d paired and stored in infra (total %d)", device_type_str(ack->hdr.device_type), dst_id, storage_infra_count());
        radio_update_known_devices();

        // Send Joined Network packet to the newly paired device
        joined_network_t jn_pkt = {
            .device_type = PRODUCT_DEVICE_TYPE,
            .device_id = radio_get_device_id(),
            .serial_num = PRODUCT_SERIAL_NUMBER,
            .version = FIRMWARE_VERSION,
        };

        if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_ANCHOR) {
            product_info_update_hop();
            jn_pkt.connected_device_id = 0xFFFF; // Anchor doesn't have connected device ID at this point, set to 0xFFFF to indicate
            jn_pkt.hop_num = PRODUCT_HOP_NUMBER;
            jn_pkt.sensor_count = storage_sensor_count();
        } else if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_SENSOR) {
            PRODUCT_CONNECTED_DEVICE_ID = dst_id;
            jn_pkt.connected_device_id = dst_id;
            jn_pkt.hop_num = 0xFF;
            jn_pkt.sensor_count = 0xFF;
        }

        uint8_t tid = tracker_next_id();
        tracker_add(dst_id, tid, PACKET_JOINED_NETWORK,
            (ack->hop_num + 1) * 500, 5, &jn_pkt, sizeof(jn_pkt));

        LOG_INF("Sending JOINED_NETWORK to %d for %s: hop %d", dst_id, device_type_str(ack->hdr.device_type), jn_pkt.hop_num);
        send_joined_network(0, &jn_pkt, dst_id, tid, STATUS_SUCCESS);
    } else if (status == STATUS_ALREADY_EXISTS) {
        LOG_INF("%s %d already paired, received PAIR_ACK with success", device_type_str(ack->hdr.device_type), dst_id);
    } else if (status == STATUS_STORAGE_FULL) {
        LOG_WRN("Storage full, cannot pair with %s %d, received PAIR_ACK with failure", device_type_str(ack->hdr.device_type), dst_id);
    }
    
    return;
}

/* Handle joined network packet */
void handle_joined_network(const joined_network_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const joined_network_t *jn = (const joined_network_t *)pkt;

    if (jn->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("JOINED_NETWORK from %s ID:%d: status 0x%02x", device_type_str(jn->hdr.device_type), dst_id, jn->hdr.status);

    if (jn->hdr.status == STATUS_SUCCESS) {
        switch(jn->hdr.device_type) {
            case DEVICE_TYPE_GATEWAY:
                LOG_WRN("Received JOINED_NETWORK from gateway device %d, ignoring", dst_id);
                break;

            case DEVICE_TYPE_ANCHOR:
                // Upstream the Packet to Gateway if Anchor receives the JOINED_NETWORK packet from Sensor
                if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_ANCHOR) {
                    LOG_INF("Forwarding JOINED_NETWORK from %s %d upstream", device_type_str(jn->device_type), dst_id);
                    uint8_t tid = tracker_next_id();
                    tracker_add(dst_id, tid, PACKET_JOINED_NETWORK,
                        5000, 5, jn, sizeof(*jn));
                    send_joined_network(0, jn, PRODUCT_CONNECTED_DEVICE_ID, tid, STATUS_SUCCESS);
                    return;
                } else if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_GATEWAY) {
                    if (check_mesh_storage(dst_id) == STATUS_SUCCESS) {
                        mesh_entry_t entry;
                        entry.device_type = jn->device_type;
                        entry.device_id = jn->device_id;
                        entry.serial_num = jn->serial_num;
                        entry.version = jn->version;
                        entry.connected_device_id = jn->connected_device_id;
                        entry.hop_num = jn->hop_num;
                        entry.sensor_count = jn->sensor_count;
                        int err = storage_mesh_add(&entry);
                        if (err) {
                            LOG_ERR("Failed to store joined mesh device, err %d", err);
                            send_joined_network_ack(0, dst_id, jn->hdr.tracking_id, STATUS_FAILURE);
                            return;
                        }
                        LOG_INF("Gateway %d successfully joined network", dst_id);
                        LOG_INF("Sending JOINED_NETWORK_ACK to %d: status 0x%02x", dst_id, STATUS_SUCCESS);
                        send_joined_network_ack(0, dst_id, jn->hdr.tracking_id, STATUS_SUCCESS);
                    }
                } else if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_SENSOR) {
                    LOG_WRN("Sensor will never receive JOINED_NETWORK from anchor, ignoring");
                    return;
                }
                break;
            case DEVICE_TYPE_SENSOR:
                LOG_INF("Sensor %d successfully joined network", dst_id);
                break;
            default:
                LOG_WRN("Unknown device type 0x%02x in JOINED_NETWORK from %d, ignoring", jn->hdr.device_type, dst_id);
                return;
        }
    } else {
        LOG_WRN("%s %d failed to join network: status 0x%02x", device_type_str(jn->hdr.device_type), dst_id, jn->hdr.status);
    }
}

/* Handle joined network acknowledgment packet */
void handle_joined_network_ack(const joined_network_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const joined_network_ack_t *ack = (const joined_network_ack_t *)pkt;

    if (ack->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("JOINED_NETWORK_ACK from %s ID:%d: status 0x%02x", device_type_str(ack->hdr.device_type), dst_id, ack->hdr.status);

    /* Find and remove the tracker by tracking ID from the packet. */
    int idx = tracker_find_by_tracking_id(ack->hdr.tracking_id);
    if (idx >= 0) {
        tracker_remove(idx);
    }

    if (ack->hdr.status == STATUS_SUCCESS) {
        LOG_INF("%s %d successfully acknowledged joined network", device_type_str(ack->hdr.device_type), dst_id);
    } else {
        LOG_WRN("%s %d failed to join network: status 0x%02x", device_type_str(ack->hdr.device_type), dst_id, ack->hdr.status);
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
static void select_and_confirm(void)
{
    if (resp_candidate_count == 0) {
        LOG_WRN("No candidates collected during window");
        return;
    }

    sort_candidates();

    /* How many slots available?
     * Sensor: only 1 upstream connection allowed.
     * Anchor: up to STORAGE_PART1_MAX_ENTRIES. */
    int max_slots = (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_SENSOR) ? 1 : STORAGE_PART1_MAX_ENTRIES;
    int available = max_slots - storage_infra_count();

    if (available <= 0) {
        LOG_WRN("Infra storage full, cannot pair with any candidates");
        return;
    }

    int to_confirm = (resp_candidate_count < available) ? resp_candidate_count : available;

    LOG_INF("Selecting %d of %d candidates (sorted by hop then RSSI)", to_confirm, resp_candidate_count);

    for (int i = 0; i < to_confirm; i++) {
        struct response_candidate *c = &resp_candidates[i];

        /* Skip if already stored. */
        uint8_t status = check_infra_storage(c->sender_id, c->device_type, true);

        if (status == STATUS_ALREADY_EXISTS) {
            LOG_INF("Candidate %s ID:%d already paired, skipping", device_type_str(c->device_type), c->sender_id);
            continue;
        }

        if (status == STATUS_STORAGE_FULL) {
            LOG_WRN("Infra storage full, stopping selection");
            break;
        }

        uint8_t tid = tracker_next_id();

        LOG_INF("Sending PAIR_CONFIRM to %s ID:%d (hop:%d, RSSI:%d.%d, tid:%d)",
            device_type_str(c->device_type), c->sender_id,
            c->hop_num, (c->rssi_2 / 2), (c->rssi_2 & 0b1) * 5, tid);

        tracker_add(c->sender_id, tid, PACKET_PAIR_CONFIRM, PAIR_TIMEOUT_MS, PAIR_MAX_RETRIES, NULL, 0);
        send_pair_confirm(0, c->sender_id, tid, STATUS_SUCCESS);
    }
}

bool mesh_is_collecting(void)
{
    return collecting;
}

void mesh_tick(void)
{
    if (!collecting) {
        return;
    }

    if (nbtimeout_expired(&collect_timer)) {
        LOG_INF("Collection window expired — %d candidates", resp_candidate_count);
        collecting = false;
        nbtimeout_stop(&collect_timer);
        select_and_confirm();
    }
}