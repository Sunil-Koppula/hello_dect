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

/* Send pairing request packet. */
int send_pair_request(uint32_t handle)
{
    uint32_t random_num = generate_random_number();
    pair_request_t packet = {
        .packet_type = PACKET_PAIR_REQUEST,
        .device_type = PRODUCT_DEVICE_TYPE,
        .priority = PACKET_PRIORITY_HIGH,
        .random_num = random_num,
    };

    return tx_queue_put(&packet, sizeof(packet), QUEUE_PRIO_HIGH);
}

/* Send pairing response packet. */
int send_pair_response(uint32_t handle, uint16_t dst_id, uint8_t status, uint32_t hash)
{
    pair_response_t packet = {
        .packet_type = PACKET_PAIR_RESPONSE,
        .device_type = PRODUCT_DEVICE_TYPE,
        .priority = PACKET_PRIORITY_HIGH,
        .device_id = dst_id,
        .status = status,
        .hash = hash,
    };

    return tx_queue_put(&packet, sizeof(packet), QUEUE_PRIO_HIGH);
}

/* Send pairing confirm packet. */
int send_pair_confirm(uint32_t handle, uint16_t dst_id, uint8_t status)
{
    pair_confirm_t packet = {
        .packet_type = PACKET_PAIR_CONFIRM,
        .device_type = PRODUCT_DEVICE_TYPE,
        .priority = PACKET_PRIORITY_HIGH,
        .device_id = dst_id,
        .status = status,
    };

    return tx_queue_put(&packet, sizeof(packet), QUEUE_PRIO_HIGH);
}

/* Send pairing acknowledgment packet. */
int send_pair_ack(uint32_t handle, uint16_t dst_id, uint8_t status)
{
    pair_ack_t packet = {
        .packet_type = PACKET_PAIR_ACK,
        .device_type = PRODUCT_DEVICE_TYPE,
        .priority = PACKET_PRIORITY_HIGH,
        .device_id = dst_id,
        .status = status,
    };

    return tx_queue_put(&packet, sizeof(packet), QUEUE_PRIO_HIGH);
}

/* Handle received pairing request packet. */
void handle_pair_request(const pair_request_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    LOG_INF("Pair Request from %s ID:%d and RSSI:%d", device_type_str(pkt->device_type), dst_id, (rssi_2 / 2));

    switch(pkt->device_type) {
        case DEVICE_TYPE_GATEWAY:
            LOG_WRN("Received pair request from gateway device %d, ignoring", dst_id);
            return;
        case DEVICE_TYPE_ANCHOR:
            // Check if device is already paired with this anchor
            for (int i = 0; i < storage_infra_count(); i++) {
                infra_entry_t entry;
                if (storage_infra_get(i, &entry) == 0 && entry.device_id == dst_id) {
                    LOG_INF("Device %d already paired, responding with ALREADY_EXISTS", dst_id);
                    send_pair_response(0, dst_id, STATUS_ALREADY_EXISTS, 0);
                    return;
                }
            }
            // Check if infra partition is full
            if (storage_infra_count() >= STORAGE_PART1_MAX_ENTRIES) {
                LOG_WRN("Infra storage full, rejecting pair request from %d", dst_id);
                send_pair_response(0, dst_id, STATUS_STORAGE_FULL, 0);
                return;
            }
            break;
        case DEVICE_TYPE_SENSOR:
            // Check if device is already paired with this gateway/anchor
            for (int i = 0; i < storage_sensor_count(); i++) {
                sensor_entry_t entry;
                if (storage_sensor_get(i, &entry) == 0 && entry.device_id == dst_id) {
                    LOG_INF("Device %d already paired, responding with ALREADY_EXISTS", dst_id);
                    send_pair_response(0, dst_id, STATUS_ALREADY_EXISTS, 0);
                    return;
                }
            }
            // Check if sensor partition is full
            if (storage_sensor_count() >= STORAGE_PART2_MAX_ENTRIES) {
                LOG_WRN("Sensor storage full, rejecting pair request from %d", dst_id);
                send_pair_response(0, dst_id, STATUS_STORAGE_FULL, 0);
                return;
            }
            break;
        default:
            LOG_WRN("Unknown device type 0x%02x in pair request from %d, rejecting", pkt->device_type, dst_id);
            return;
    }

    uint32_t hash = compute_pair_hash(dst_id, pkt->random_num);

    send_pair_response(0, dst_id, STATUS_SUCCESS, hash);
}