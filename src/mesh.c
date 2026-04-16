/*
 * Mesh protocol shared utilities for DECT NR+ mesh network
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include "mesh.h"
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
static uint8_t check_infra_storage(uint16_t device_id, uint8_t device_type)
{
    infra_entry_t entry;

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
            status = check_infra_storage(dst_id, pkt->hdr.device_type);
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
            status = check_infra_storage(dst_id, pkt->hdr.device_type);
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