/* Mesh TX Helpers */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include "main_sub.h"
#include "mesh.h"
#include "protocol.h"
#include "tracker.h"
#include "product_info.h"
#include "radio.h"
#include "queue.h"

LOG_MODULE_REGISTER(mesh_tx, CONFIG_MESH_TX_LOG_LEVEL);

/* Mesh time — milliseconds since the gateway started.
 * mesh_time_get() returns mesh_time_offset + k_uptime_get(), so the value
 * keeps advancing locally between syncs. On the gateway, the offset is set
 * once at startup. On anchors/sensors, the offset is rewritten each time
 * a SYNC_TIME packet arrives.
 */
static int64_t mesh_time_offset;

/* Generate random number for pairing. */
static uint32_t generate_random_number(void)
{
    uint32_t random_num;
    sys_rand_get(&random_num, sizeof(random_num));
    return random_num;
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------------**** TX Helpers ****------------------------------------------------------------------------------- */

/* Send pairing request packet. */
int send_pair_request(void)
{
    uint32_t random_num = generate_random_number();
    uint32_t hash = compute_pair_hash(radio_get_device_id(), random_num);

    pair_request_t packet = {
        .hdr = {
            .packet_type = PACKET_PAIR_REQUEST,
            .device_type = DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracker_next_id(),
            .device_id = 0,
            .status = STATUS_SUCCESS,
        },
        .random_num = random_num,
        .hash = hash,
    };

    // Add tracker entry for retries
    tracker_add(radio_get_device_id(), 0, packet.hdr.tracking_id, PACKET_PAIR_REQUEST, 5 * PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF("----> Sending PAIR_REQUEST");
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send pairing response packet. */
int send_pair_response(uint16_t dst_id,uint8_t dst_type ,uint8_t tracking_id, uint8_t status)
{
    pair_response_t packet = {
        .hdr = {
            .packet_type = PACKET_PAIR_RESPONSE,
            .device_type = DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .hop_num = DEVICE_HOP_NUMBER,
    };

    LOG_INF("----> Sending PAIR_RESPONSE to device %s ID:%d with hop_num %d (status: 0x%02x)", device_type_str(dst_type), dst_id, DEVICE_HOP_NUMBER, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send pairing confirm packet. */
int send_pair_confirm(uint16_t dst_id, uint8_t dst_type, uint8_t status)
{

    pair_confirm_t packet = {
        .hdr = {
            .packet_type = PACKET_PAIR_CONFIRM,
            .device_type = DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracker_next_id(),
            .device_id = dst_id,
            .status = status,
        },
        .version = FIRMWARE_VERSION,
        .hop_num = DEVICE_HOP_NUMBER,
    };

    // Add tracker entry for retries
    tracker_add(dst_id, radio_get_device_id(), packet.hdr.tracking_id, PACKET_PAIR_CONFIRM, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF("----> Sending PAIR_CONFIRM to device %s ID:%d with hop_num %d (status: 0x%02x)", device_type_str(dst_type), dst_id, DEVICE_HOP_NUMBER, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send pairing acknowledgment packet. */
int send_pair_ack(uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status)
{
    pair_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_PAIR_ACK,
            .device_type = DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .hop_num = DEVICE_HOP_NUMBER,
    };

    LOG_INF("----> Sending PAIR_ACK to device %s ID:%d with hop_num %d (status: 0x%02x)", device_type_str(dst_type), dst_id, DEVICE_HOP_NUMBER, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send joined network packet. */
int send_joined_network(const joined_network_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t status)
{
    joined_network_t packet = {
        .hdr = {
            .packet_type = PACKET_JOINED_NETWORK,
            .device_type = DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracker_next_id(),
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

    // Add tracker entry for retries
    tracker_add(dst_id, radio_get_device_id(), packet.hdr.tracking_id, PACKET_JOINED_NETWORK, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF("----> Sending JOINED_NETWORK to device %s ID:%d for device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, device_type_str(pkt->device_type), pkt->device_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send joined network acknowledgment packet. */
int send_joined_network_ack(const joined_network_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status)
{
    joined_network_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_JOINED_NETWORK_ACK,
            .device_type = DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .dst_device_id = pkt->dst_device_id,
        .dst_device_type = pkt->dst_device_type,
    };

    LOG_INF("----> Sending JOINED_NETWORK_ACK to device %s ID:%d for device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, device_type_str(pkt->dst_device_type), pkt->dst_device_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send ping device packet. */
int send_ping_device(uint16_t dst_id, uint8_t dst_type, uint8_t status)
{
    ping_device_t packet = {
        .hdr = {
            .packet_type = PACKET_PING_DEVICE,
            .device_type = DEVICE_TYPE,
            .priority = PACKET_PRIORITY_LOW,
            .tracking_id = tracker_next_id(),
            .device_id = dst_id,
            .status = status,
        },
        .hop_num = DEVICE_HOP_NUMBER,
        .version = FIRMWARE_VERSION,
        .timestamp = mesh_time_get() + 15,
    };

    // Add tracker entry for retries
    tracker_add(dst_id, radio_get_device_id(), packet.hdr.tracking_id, PACKET_PING_DEVICE, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF("----> Sending PING_DEVICE to device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send ping acknowledgment packet. */
int send_ping_ack(uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status)
{
    ping_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_PING_ACK,
            .device_type = DEVICE_TYPE,
            .priority = PACKET_PRIORITY_LOW,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .hop_num = DEVICE_HOP_NUMBER,
        .version = FIRMWARE_VERSION,
        .timestamp = mesh_time_get() + 15,
    };

    LOG_INF("----> Sending PING_ACK to device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send device updated packet. */
int send_device_updated(const device_updated_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t status)
{
    device_updated_t packet = {
        .hdr = {
            .packet_type = PACKET_DEVICE_UPDATED,
            .device_type = DEVICE_TYPE,
            .priority = PACKET_PRIORITY_LOW,
            .tracking_id = tracker_next_id(),
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

    // Add tracker entry for retries
    tracker_add(dst_id, radio_get_device_id(), packet.hdr.tracking_id, PACKET_DEVICE_UPDATED, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF("----> Sending DEVICE_UPDATED to device %s ID:%d for device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, device_type_str(pkt->device_type), pkt->device_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send device updated acknowledgment packet. */
int send_device_updated_ack(const device_updated_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status)
{
    device_updated_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_DEVICE_UPDATED_ACK,
            .device_type = DEVICE_TYPE,
            .priority = PACKET_PRIORITY_LOW,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .dst_device_id = pkt->dst_device_id,
        .dst_device_type = pkt->dst_device_type,
    };

    LOG_INF("----> Sending DEVICE_UPDATED_ACK to device %s ID:%d for device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, device_type_str(pkt->dst_device_type), pkt->dst_device_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send repair request packet. */
int send_repair_request(uint16_t dst_id, uint8_t dst_type)
{
    uint32_t random_num = generate_random_number();
    uint32_t hash = compute_pair_hash(radio_get_device_id(), random_num);

    repair_request_t packet = {
        .hdr = {
            .packet_type = PACKET_REPAIR_REQUEST,
            .device_type = DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracker_next_id(),
            .device_id = dst_id,
            .status = STATUS_SUCCESS,
        },
        .random_num = random_num,
        .hash = hash,
        .version = FIRMWARE_VERSION,
        .hop_num = DEVICE_HOP_NUMBER,
    };

    // Add tracker entry for retries
    tracker_add(dst_id, radio_get_device_id(), packet.hdr.tracking_id, PACKET_REPAIR_REQUEST, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF("----> Sending REPAIR_REQUEST to device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, STATUS_SUCCESS);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send repair response packet. */
int send_repair_response(uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status)
{
    repair_response_t packet = {
        .hdr = {
            .packet_type = PACKET_REPAIR_RESPONSE,
            .device_type = DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .version = FIRMWARE_VERSION,
        .hop_num = DEVICE_HOP_NUMBER,
    };

    LOG_INF("----> Sending REPAIR_RESPONSE to device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send route info packet. */
int send_route_info(const route_info_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t status)
{
    route_info_t packet = {
        .hdr = {
            .packet_type = PACKET_ROUTE_INFO,
            .device_type = DEVICE_TYPE,
            .priority = PACKET_PRIORITY_MEDIUM,
            .tracking_id = tracker_next_id(),
            .device_id = dst_id,
            .status = status,
        },
        .device_id = pkt->device_id,
        .device_type = pkt->device_type,
        .route_len = pkt->route_len,
        .avg_rssi_2 = pkt->avg_rssi_2,
    };

    // Add tracker entry for retries
    tracker_add(dst_id, radio_get_device_id(), packet.hdr.tracking_id, PACKET_ROUTE_INFO, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF("----> Sending ROUTE_INFO to device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send route info acknowledgment packet. */
int send_route_info_ack(const route_info_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status)
{
    route_info_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_ROUTE_INFO_ACK,
            .device_type = DEVICE_TYPE,
            .priority = PACKET_PRIORITY_MEDIUM,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .device_id = pkt->device_id,
        .device_type = pkt->device_type,
        .route_len = pkt->route_len,
        .avg_rssi_2 = pkt->avg_rssi_2,
    };

    LOG_INF("----> Sending ROUTE_INFO_ACK to device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* --------------------------------------------------------------------------**** Mesh time accessors ****--------------------------------------------------------------------------- */

void mesh_time_init(void)
{
    /* Gateway-only: anchor mesh_time at 0 for the moment of startup. */
    mesh_time_offset = -k_uptime_get();
}

uint64_t mesh_time_get(void)
{
    return (uint64_t)(mesh_time_offset + k_uptime_get());
}

void mesh_time_set(uint64_t remote_mesh_time)
{
    mesh_time_offset = (int64_t)remote_mesh_time - k_uptime_get();
}