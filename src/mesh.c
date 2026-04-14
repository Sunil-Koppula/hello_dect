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
#include "protocol.h"
#include "product_info.h"

LOG_MODULE_REGISTER(mesh, CONFIG_MESH_LOG_LEVEL);

/* Send pairing request packet. */
int send_pair_request(uint32_t handle, uint32_t random_num)
{
    pair_request_t packet = {
        .packet_type = PACKET_PAIR_REQUEST,
        .device_type = PRODUCT_DEVICE_TYPE,
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
        .device_id = dst_id,
        .status = status,
    };

    return tx_queue_put(&packet, sizeof(packet), QUEUE_PRIO_HIGH);
}

/* Handle received pairing request packet. */
void handle_pair_request(const pair_request_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    LOG_INF("Pair Request from %s ID:%d and RSSI:%d", device_type_str(pkt->device_type), dst_id, (rssi_2 / 2));

    //ToDo
    // Check if device is already paired or not, and respond accordingly

    // For demo, we simply respond with success to all pairing requests
    uint32_t hash = 0xDEADBEEF; // In real implementation, use a proper hash of device info and random number
    send_pair_response(0, dst_id, STATUS_SUCCESS, hash);
}