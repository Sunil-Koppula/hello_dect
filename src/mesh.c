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

/* Handle received pairing request packet. */
void handle_pair_request(const pair_request_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    LOG_INF("Pair Request from %s ID:%d and RSSI:%d", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2));

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

    send_pair_response(0, dst_id, pkt->hdr.tracking_id, status, hash, 0);
}

/* Handle received pairing response packet. */
void handle_pair_response(const pair_response_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const pair_response_t *resp = (const pair_response_t *)pkt;

    if (resp->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("PAIR_RESPONSE from %s ID:%d: status 0x%02x, hop %d", device_type_str(resp->hdr.device_type), dst_id, resp->hdr.status, resp->hop_num);

    /* Find and remove the request tracker by tracking ID from the packet. */
    int idx = tracker_find_by_tracking_id(resp->hdr.tracking_id);
    if (idx >= 0) {
        tracker_remove(idx);
    }

    uint8_t status;

    if (resp->hdr.status == STATUS_SUCCESS) {
        switch (resp->hdr.device_type) {
            case DEVICE_TYPE_GATEWAY:
            case DEVICE_TYPE_ANCHOR:
            {
                switch(PRODUCT_DEVICE_TYPE) {
                    case DEVICE_TYPE_GATEWAY:
                        LOG_WRN("Gateway will never receive PAIR_RESPONSE, ignoring");
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
                        LOG_WRN("Unknown device type 0x%02x in pair response from %d, ignoring", pkt->hdr.device_type, dst_id);
                        return;
                }
                break;
            }
            case DEVICE_TYPE_SENSOR:
                LOG_WRN("Received PAIR_RESPONSE from sensor device %d, ignoring", dst_id);
                return;
            default:
                LOG_WRN("Unknown device type 0x%02x in pair response from %d, rejecting", resp->hdr.device_type, dst_id);
                return;
        }
    } else {
        LOG_WRN("PAIR_RESPONSE failed: status 0x%02x", resp->hdr.status);
        return;
    }

    if (status == STATUS_SUCCESS) {
        // Send confirm with a new tracking ID.
        uint8_t tid = tracker_next_id();
        LOG_INF("Sending PAIR_CONFIRM to %s ID:%d (tid: %d)", device_type_str(resp->hdr.device_type), dst_id, tid);
        tracker_add(dst_id, tid, PACKET_PAIR_CONFIRM, PAIR_TIMEOUT_MS, PAIR_MAX_RETRIES);
        send_pair_confirm(0, dst_id, tid, STATUS_SUCCESS);
    } else if (status == STATUS_ALREADY_EXISTS) {
        LOG_INF("%s %d already paired, sending PAIR_CONFIRM with success", device_type_str(resp->hdr.device_type), dst_id);
    } else if (status == STATUS_STORAGE_FULL) {
        LOG_WRN("Storage full, cannot pair with %s %d, sending PAIR_CONFIRM with failure", device_type_str(resp->hdr.device_type), dst_id);
    }

    return;
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
            }
            break;
        default:
            LOG_WRN("Unknown device type 0x%02x in PAIR_CONFIRM from %d, rejecting", pkt->hdr.device_type, dst_id);
            return;
    }

    send_pair_ack(0, dst_id, conf->hdr.tracking_id, status, 0);
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

    if (ack->hdr.status == STATUS_SUCCESS) {
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
        entry.hop_num = 0xFF;
        entry.rssi_2 = rssi_2;
        int err = storage_infra_add(&entry);
        if (err) {
            LOG_ERR("Failed to store paired device, err %d", err);
            return;
        }
        LOG_INF("Device %d paired and stored in infra (total %d)", dst_id, storage_infra_count());
    } else if (status == STATUS_ALREADY_EXISTS) {
        LOG_INF("%s %d already paired, received PAIR_ACK with success", device_type_str(ack->hdr.device_type), dst_id);
    } else if (status == STATUS_STORAGE_FULL) {
        LOG_WRN("Storage full, cannot pair with %s %d, received PAIR_ACK with failure", device_type_str(ack->hdr.device_type), dst_id);
    }
    
    return;
}