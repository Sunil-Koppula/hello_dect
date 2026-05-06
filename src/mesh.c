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
#include "data.h"

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
static struct nbtimeout collect_timer;
static bool collecting;

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
            return STATUS_ALREADY_EXISTS;
        }
        
        if (storage_infra_count() >= 1) {
            return STATUS_STORAGE_FULL;
        }

        return STATUS_SUCCESS;
    }

    for (int i = 0; i < storage_infra_count(); i++) {
        if (storage_infra_get(i, &entry) == 0 && entry.device_id == device_id) {
            return STATUS_ALREADY_EXISTS;
        }
    }

    if (storage_infra_count() >= MAX_ANCHORS) {
        return STATUS_STORAGE_FULL;
    }

    return STATUS_SUCCESS;
}

/* Update Infra Storage */
static bool update_infra_storage(uint16_t device_id, uint8_t hop_num, int16_t rssi_2)
{
    uint8_t current_hop_num = PRODUCT_HOP_NUMBER;
    infra_entry_t entry;
    for (int i = 0; i < storage_infra_count(); i++) {
        if (storage_infra_get(i, &entry) == 0 && entry.device_id == device_id) {
            entry.rssi_2 = rssi_2;
            if (hop_num != 0xFF && entry.hop_num != hop_num) {
                entry.hop_num = hop_num;
            }
            if (entry.version != FIRMWARE_VERSION) {
                entry.version = FIRMWARE_VERSION;
            }
            int err = storage_infra_update(i, &entry);
            if (err) {
                LOG_ERR("Failed to update infra entry for device %d, err %d", device_id, err);
                return false;
            }
            product_info_update_hop();
            break;
        }
    }
    if (current_hop_num != PRODUCT_HOP_NUMBER) {
        return true;
    }
    return false;
}

/* Check Sensor Storage */
static uint8_t check_sensor_storage(uint16_t device_id)
{
    sensor_entry_t entry;

    for (int i = 0; i < storage_sensor_count(); i++) {
        if (storage_sensor_get(i, &entry) == 0 && entry.device_id == device_id) {
            return STATUS_ALREADY_EXISTS;
        }
    }

    if (storage_sensor_count() >= MAX_SENSORS) {
        return STATUS_STORAGE_FULL;
    }

    return STATUS_SUCCESS;
}

/* Update Sensor Storage */
static void update_sensor_storage(uint16_t device_id, uint16_t version)
{
    sensor_entry_t entry;
    for (int i = 0; i < storage_sensor_count(); i++) {
        if (storage_sensor_get(i, &entry) == 0 && entry.device_id == device_id) {
            if (entry.version == version) {
                return;
            }
            entry.version = version;
            int err = storage_sensor_update(i, &entry);
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
            .device_type = PRODUCT_DEVICE_TYPE,
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
int send_pair_response(uint16_t dst_id, uint8_t tracking_id, uint8_t status, uint8_t hop_num)
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
        .hop_num = hop_num,
    };

    LOG_INF("----> Sending PAIR_RESPONSE to device %s ID:%d with hop_num %d", device_type_str(dst_id), dst_id, hop_num);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send pairing confirm packet. */
int send_pair_confirm(uint16_t dst_id, uint8_t status)
{

    pair_confirm_t packet = {
        .hdr = {
            .packet_type = PACKET_PAIR_CONFIRM,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracker_next_id(),
            .device_id = dst_id,
            .status = status,
        },
        .version = FIRMWARE_VERSION,
    };

    // Add tracker entry for retries
    tracker_add(dst_id, radio_get_device_id(), packet.hdr.tracking_id, PACKET_PAIR_CONFIRM, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF("----> Sending PAIR_CONFIRM to device %s ID:%d", device_type_str(dst_id), dst_id);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send pairing acknowledgment packet. */
int send_pair_ack(uint16_t dst_id, uint8_t tracking_id, uint8_t status, uint8_t hop_num)
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

    LOG_INF("----> Sending PAIR_ACK to device %s ID:%d with hop_num %d", device_type_str(dst_id), dst_id, hop_num);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send joined network packet. */
int send_joined_network(const joined_network_t *pkt, uint16_t dst_id, uint8_t status)
{
    joined_network_t packet = {
        .hdr = {
            .packet_type = PACKET_JOINED_NETWORK,
            .device_type = PRODUCT_DEVICE_TYPE,
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

    LOG_INF("----> Sending JOINED_NETWORK to device %s ID:%d for device %s ID:%d", device_type_str(dst_id), dst_id, device_type_str(pkt->device_id), pkt->device_id);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send joined network acknowledgment packet. */
int send_joined_network_ack(uint16_t dst_device_id, uint16_t dst_id, uint8_t tracking_id, uint8_t status)
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
        .dst_device_id = dst_device_id,
    };

    LOG_INF("----> Sending JOINED_NETWORK_ACK to device %s ID:%d for device %s ID:%d", device_type_str(dst_id), dst_id, device_type_str(dst_device_id), dst_device_id);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send ping device packet. */
int send_ping_device(uint16_t dst_id, uint8_t status)
{
    ping_device_t packet = {
        .hdr = {
            .packet_type = PACKET_PING_DEVICE,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_LOW,
            .tracking_id = tracker_next_id(),
            .device_id = dst_id,
            .status = status,
        },
        .hop_num = PRODUCT_HOP_NUMBER,
        .version = FIRMWARE_VERSION,
        .timestamp = mesh_time_get() + 15,
    };

    // Add tracker entry for retries
    tracker_add(dst_id, radio_get_device_id(), packet.hdr.tracking_id, PACKET_PING_DEVICE, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF("----> Sending PING_DEVICE to device %s ID:%d", device_type_str(dst_id), dst_id);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send ping acknowledgment packet. */
int send_ping_ack(uint16_t dst_id, uint8_t tracking_id, uint8_t status)
{
    ping_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_PING_ACK,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_LOW,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .hop_num = PRODUCT_HOP_NUMBER,
        .version = FIRMWARE_VERSION,
        .timestamp = mesh_time_get() + 15,
    };

    LOG_INF("----> Sending PING_ACK to device %s ID:%d", device_type_str(dst_id), dst_id);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send device updated packet. */
int send_device_updated(const device_updated_t *pkt, uint16_t dst_id, uint8_t status)
{
    device_updated_t packet = {
        .hdr = {
            .packet_type = PACKET_DEVICE_UPDATED,
            .device_type = PRODUCT_DEVICE_TYPE,
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

    LOG_INF("----> Sending DEVICE_UPDATED to device %s ID:%d for device %s ID:%d", device_type_str(dst_id), dst_id, device_type_str(pkt->device_id), pkt->device_id);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send device updated acknowledgment packet. */
int send_device_updated_ack(uint16_t dst_device_id, uint16_t dst_id, uint8_t tracking_id, uint8_t status)
{
    device_updated_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_DEVICE_UPDATED_ACK,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_LOW,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .dst_device_id = dst_device_id,
    };

    LOG_INF("----> Sending DEVICE_UPDATED_ACK to device %s ID:%d for device %s ID:%d", device_type_str(dst_id), dst_id, device_type_str(dst_device_id), dst_device_id);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send repair request packet. */
int send_repair_request(uint16_t dst_id)
{
    uint32_t random_num = generate_random_number();
    uint32_t hash = compute_pair_hash(radio_get_device_id(), random_num);

    repair_request_t packet = {
        .hdr = {
            .packet_type = PACKET_REPAIR_REQUEST,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracker_next_id(),
            .device_id = dst_id,
            .status = STATUS_SUCCESS,
        },
        .random_num = random_num,
        .hash = hash,
        .version = FIRMWARE_VERSION,
        .hop_num = PRODUCT_HOP_NUMBER,
    };

    // Add tracker entry for retries
    tracker_add(dst_id, radio_get_device_id(), packet.hdr.tracking_id, PACKET_REPAIR_REQUEST, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF("----> Sending REPAIR_REQUEST to device %s ID:%d", device_type_str(dst_id), dst_id);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send repair response packet. */
int send_repair_response(uint16_t dst_id, uint8_t tracking_id, uint8_t status)
{
    repair_response_t packet = {
        .hdr = {
            .packet_type = PACKET_REPAIR_RESPONSE,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .version = FIRMWARE_VERSION,
        .hop_num = PRODUCT_HOP_NUMBER,
    };

    LOG_INF("----> Sending REPAIR_RESPONSE to device %s ID:%d", device_type_str(dst_id), dst_id);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send route info packet. */
int send_route_info(const route_info_t *pkt, uint16_t dst_id, uint8_t status)
{
    route_info_t packet = {
        .hdr = {
            .packet_type = PACKET_ROUTE_INFO,
            .device_type = PRODUCT_DEVICE_TYPE,
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

    LOG_INF("----> Sending ROUTE_INFO to device %s ID:%d", device_type_str(dst_id), dst_id);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send route info acknowledgment packet. */
int send_route_info_ack(const route_info_ack_t *pkt, uint16_t dst_id, uint8_t tracking_id, uint8_t status)
{
    route_info_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_ROUTE_INFO_ACK,
            .device_type = PRODUCT_DEVICE_TYPE,
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

    LOG_INF("----> Sending ROUTE_INFO_ACK to device %s ID:%d", device_type_str(dst_id), dst_id);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------**** Handlers Functions ****--------------------------------------------------------------------------- */

/* Handle received pairing request packet. */
void handle_pair_request(const pair_request_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    LOG_INF("----> Recieved PAIR_REQUEST from %s ID:%d and RSSI:%d (status: %d)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    /* Verify hash: hash should equal compute_pair_hash(sender_id, random_num). */
    uint32_t expected_hash = compute_pair_hash(dst_id, pkt->random_num);

    if (pkt->hash != expected_hash) {
        LOG_WRN("PAIR_REQUEST hash mismatch from %d (got 0x%08x, expected 0x%08x)", dst_id, pkt->hash, expected_hash);
        send_pair_response(dst_id, pkt->hdr.tracking_id, STATUS_AUTH_FAILED, 0);
        return;
    }

    uint8_t status;

    switch(pkt->hdr.device_type) {
        case DEVICE_TYPE_GATEWAY:
            return;
        case DEVICE_TYPE_ANCHOR:
            // Check if device is already paired with this anchor
            status = check_infra_storage(dst_id, pkt->hdr.device_type, true);
            break;
        case DEVICE_TYPE_SENSOR:
            // Check if device is already paired with this gateway/anchor
            status = check_sensor_storage(dst_id);
            // Testing only
            if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_GATEWAY) {
                return;
            }
            break;
        default:
            LOG_WRN("Unknown device type 0x%02x in PAIR_REQUEST from %d, rejecting", pkt->hdr.device_type, dst_id);
            return;
    }

    send_pair_response(dst_id, pkt->hdr.tracking_id, status, PRODUCT_HOP_NUMBER);
}

/* Handle received pairing response packet.
 * Collects candidates for 3 seconds, then mesh_tick() selects the best ones. */
void handle_pair_response(const pair_response_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const pair_response_t *resp = (const pair_response_t *)pkt;

    if (resp->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("----> Recieved PAIR_RESPONSE from %s ID:%d and RSSI:%d (status: %d)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    /* Remove tracker entry for the pair request. */
    tracker_remove_by_tracking_id(resp->hdr.tracking_id);

    if (resp->hdr.status != STATUS_SUCCESS && resp->hdr.status != STATUS_ALREADY_EXISTS) {
        return;
    }

    if (resp->hdr.status == STATUS_ALREADY_EXISTS) {
        // Check if we already have this device in storage
        if (resp->hdr.device_type == DEVICE_TYPE_SENSOR) {
            if (check_sensor_storage(dst_id) == STATUS_SUCCESS) {
                // send repair request
                send_repair_request(dst_id);
            }
        } else {
            if (check_infra_storage(dst_id, resp->hdr.device_type, true) == STATUS_SUCCESS) {
                // send repair request
                send_repair_request(dst_id);
            }
        }
        return;
    }

    /* Validate responder type. */
    if (resp->hdr.device_type != DEVICE_TYPE_GATEWAY &&
        resp->hdr.device_type != DEVICE_TYPE_ANCHOR) {
        LOG_WRN("Unknown device type 0x%02x in PAIR_RESPONSE from %d, rejecting", pkt->hdr.device_type, dst_id);
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
        };
        LOG_INF("Candidate %d: %s ID:%d hop:%d RSSI:%d.%d", resp_candidate_count, device_type_str(resp->hdr.device_type), dst_id, resp->hop_num, (rssi_2 / 2), (rssi_2 & 0b1) * 5);
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

    LOG_INF("----> Recieved PAIR_CONFIRM from %s ID:%d and RSSI:%d (status: %d)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    if((conf->hdr.device_type != DEVICE_TYPE_SENSOR) && (conf->hdr.device_type != DEVICE_TYPE_ANCHOR)) {
        return;
    } else if (conf->hdr.status != STATUS_SUCCESS) {
        return;
    }

    uint8_t status;

    switch(pkt->hdr.device_type) {
        case DEVICE_TYPE_GATEWAY:
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
                entry.version = conf->version;
                int err = storage_infra_add(&entry);
                if (err) {
                    LOG_ERR("Failed to store paired anchor, err %d", err);
                    return;
                }
                LOG_INF("ANCHOR ID:%d paired and stored in infra (total %d)", dst_id, storage_infra_count());
                update_known_devices();
            } else if (status == STATUS_ALREADY_EXISTS) {
                LOG_INF("ANCHOR ID:%d already paired, received PAIR_CONFIRM with success", dst_id);
            }
            break;
        case DEVICE_TYPE_SENSOR:
            // Check if device is already paired with this gateway/anchor
            status = check_sensor_storage(dst_id);
            if (status == STATUS_SUCCESS) {
                sensor_entry_t entry;
                entry.device_id = dst_id;
                entry.version = conf->version;
                int err = storage_sensor_add(&entry);
                if (err) {
                    LOG_ERR("Failed to store paired sensor, err %d", err);
                    return;
                }
                LOG_INF("SENSOR ID:%d paired and stored (total %d)", dst_id, storage_sensor_count());
                update_known_devices();

                if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_ANCHOR) {
                    // Send Device Updated Packet to Gateway about new paired sensor
                    
                    device_updated_t pkt = {
                    .device_type = PRODUCT_DEVICE_TYPE,
                    .device_id = radio_get_device_id(),
                    .serial_num = PRODUCT_SERIAL_NUMBER,
                    .version = FIRMWARE_VERSION,
                    .connected_device_id = 0xFFFF,
                    .hop_num = PRODUCT_HOP_NUMBER,
                    .sensor_count = storage_sensor_count(),
                    };
                    infra_entry_t entry;
                    storage_infra_get(0, &entry);
                    send_device_updated(&pkt, entry.device_id, STATUS_SUCCESS);
                }
            } else if (status == STATUS_ALREADY_EXISTS) {
                LOG_INF("SENSOR ID:%d already paired, received PAIR_CONFIRM with success", dst_id);
            }
            break;
        default:
            LOG_WRN("Unknown device type 0x%02x in PAIR_CONFIRM from %d, rejecting", pkt->hdr.device_type, dst_id);
            return;
    }
    send_pair_ack(dst_id, conf->hdr.tracking_id, status, PRODUCT_HOP_NUMBER);

    // Send Route Info Request to the newly paired device if status is success
    if (status == STATUS_SUCCESS) {
        switch (PRODUCT_DEVICE_TYPE) {
            case DEVICE_TYPE_GATEWAY:
            {
                int err = storage_route_add(dst_id, dst_id, conf->hdr.device_type, 1, rssi_2);
                if (err) {
                    LOG_ERR("Failed to add route for device %s ID:%d, err %d", device_type_str(conf->hdr.device_type), dst_id, err);
                    return;
                }
                LOG_INF("Added route for device %s ID:%d", device_type_str(conf->hdr.device_type), dst_id);
            }
            break;

            case DEVICE_TYPE_ANCHOR:
            {
                int err = storage_route_add(dst_id, dst_id, conf->hdr.device_type, 1, rssi_2);
                if (err) {
                    LOG_ERR("Failed to add route for device %s ID:%d, err %d", device_type_str(conf->hdr.device_type), dst_id, err);
                    return;
                }
                LOG_INF("Added route for device %s ID:%d", device_type_str(conf->hdr.device_type), dst_id);

                infra_entry_t entry;
                uint16_t next_hop_id[MAX_ANCHORS];
                uint8_t next_hop_count = 0;

                for (int i = 0; i < storage_infra_count(); i++) {
                    storage_infra_get(i, &entry);
                    if ((entry.device_type == DEVICE_TYPE_ANCHOR || entry.device_type == DEVICE_TYPE_GATEWAY) && entry.hop_num <= PRODUCT_HOP_NUMBER && entry.device_id != dst_id) {
                        next_hop_id[next_hop_count++] = entry.device_id;
                    }
                }

                route_info_t forward_pkt = {
                    .device_id = dst_id,
                    .device_type = conf->hdr.device_type,
                    .route_len = 1,
                    .avg_rssi_2 = rssi_2,
                };
                for (int i = 0; i < next_hop_count; i++) {
                    send_route_info(&forward_pkt, next_hop_id[i], STATUS_DEVICE_JOINED);
                }
            }
            break;

            default:
                break; 
        }
    }
    return;

}

/* Handle received pairing acknowledgment packet. */
void handle_pair_ack(const pair_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
   const pair_ack_t *ack = (const pair_ack_t *)pkt;

    if (ack->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("----> Recieved PAIR_ACK from %s ID:%d and RSSI:%d (status: %d)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    /* Remove the confirm tracker. */
    tracker_remove_by_tracking_id(ack->hdr.tracking_id);

    uint8_t status;

    if (ack->hdr.status == STATUS_SUCCESS || ack->hdr.status == STATUS_ALREADY_EXISTS) {
        switch (ack->hdr.device_type) {
            case DEVICE_TYPE_GATEWAY:
            case DEVICE_TYPE_ANCHOR:
            {
                switch(PRODUCT_DEVICE_TYPE) {
                    case DEVICE_TYPE_GATEWAY:
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
                        LOG_WRN("Unknown device type 0x%02x in PAIR_ACK from %d, ignoring", ack->hdr.device_type, dst_id);
                        return;
                }
                break;
            }
            case DEVICE_TYPE_SENSOR:
                return;
            default:
                LOG_WRN("Unknown device type 0x%02x in PAIR_ACK from %d, ignoring", ack->hdr.device_type, dst_id);
                return;
        }
    } else {
        return;
    }

    if (status == STATUS_SUCCESS) {
        infra_entry_t entry;
        entry.device_id = dst_id;
        entry.device_type = ack->hdr.device_type;
        entry.hop_num = ack->hop_num;
        entry.rssi_2 = rssi_2;
        entry.version = ack->version;
        int err = storage_infra_add(&entry);
        if (err) {
            LOG_ERR("Failed to store paired device, err %d", err);
            return;
        }
        LOG_INF("%s ID:%d paired and stored in infra (total %d)", device_type_str(ack->hdr.device_type), dst_id, storage_infra_count());
        update_known_devices();

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

        if (storage_infra_count() == 1) {
            send_joined_network(&jn_pkt, dst_id, STATUS_SUCCESS);
        }
    } else if (status == STATUS_ALREADY_EXISTS) {
        LOG_INF("%s ID:%d already paired, received PAIR_ACK with success", device_type_str(ack->hdr.device_type), dst_id);
    } else if (status == STATUS_STORAGE_FULL) {
        LOG_WRN("Storage full, cannot pair with %s ID:%d, received PAIR_ACK with failure", device_type_str(ack->hdr.device_type), dst_id);
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

    LOG_INF("----> Received JOINED_NETWORK from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(jn->hdr.device_type), dst_id, device_type_str(jn->device_id), jn->device_id, (rssi_2 / 2), jn->hdr.status);

    if (jn->hdr.status == STATUS_SUCCESS) {
        switch(jn->hdr.device_type) {
            case DEVICE_TYPE_GATEWAY:
                break;

            case DEVICE_TYPE_ANCHOR:
            case DEVICE_TYPE_SENSOR:
                // Upstream the Packet to Gateway if Anchor receives the JOINED_NETWORK packet from Sensor
                if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_ANCHOR) {
                    // First send ACK then upstream the JOINED_NETWORK packet to gateway
                    send_joined_network_ack(jn->device_id, dst_id, jn->hdr.tracking_id, STATUS_SUCCESS);
                    LOG_INF("Forwarding JOINED_NETWORK from %s ID:%d  for %s ID:%d upstream", device_type_str(jn->hdr.device_type), dst_id, device_type_str(jn->device_id), jn->device_id);

                    infra_entry_t entry;
                    storage_infra_get(0, &entry);
                    
                    send_joined_network(jn, entry.device_id, STATUS_SUCCESS);
                    return;
                } else if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_GATEWAY) {
                    int status = check_mesh_storage(jn->device_id);
                    if (status == STATUS_SUCCESS) {
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
                            send_joined_network_ack(jn->device_id, dst_id, jn->hdr.tracking_id, STATUS_FAILURE);
                            return;
                        }
                        LOG_INF("%s ID:%d successfully joined network", device_type_str(jn->device_type), jn->device_id);
                    }
                    send_joined_network_ack(jn->device_id, dst_id, jn->hdr.tracking_id, status);
                } else if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_SENSOR) {
                    return;
                }
                break;
            default:
                LOG_WRN("Unknown device type 0x%02x in JOINED_NETWORK from %d, ignoring", jn->hdr.device_type, dst_id);
                return;
        }
    }
    return;
}

/* Handle joined network acknowledgment packet */
void handle_joined_network_ack(const joined_network_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const joined_network_ack_t *ack = (const joined_network_ack_t *)pkt;

    if (ack->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("----> Received JOINED_NETWORK_ACK from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(ack->hdr.device_type), dst_id, device_type_str(ack->dst_device_id), ack->dst_device_id, (rssi_2 / 2), ack->hdr.status);

    /* Remove the joined network tracker. */
    tracker_remove_by_tracking_id(ack->hdr.tracking_id);

    switch (PRODUCT_DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
            return;
        
        case DEVICE_TYPE_ANCHOR:
            return;

        case DEVICE_TYPE_SENSOR:
            // For Testing Purpose: Send Data Init Packet to Connected Device
            sender.dst_id = dst_id;
            data_init_t data_pkt = {
                .gen_device_id = sender.gen_device_id,
                .total_size = sender.total_size,
                .chunk_count = sender.chunk_count,
                .last_chunk_size = sender.last_chunk_size,
                .crc32 = sender.crc32,
            };
            send_data_init(dst_id, sender.priority, &data_pkt);
            return;

        default:
            LOG_WRN("Unknown device type 0x%02x in JOINED_NETWORK_ACK from %d, ignoring", ack->hdr.device_type, dst_id);
            return;
    }
}

/* Handle ping device packet */
void handle_ping_device(const ping_device_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const ping_device_t *ping = (const ping_device_t *)pkt;

    if (ping->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("----> Received PING_DEVICE from %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(ping->hdr.device_type), dst_id, (rssi_2 / 2), ping->hdr.status);

    if (ping->hdr.device_type != DEVICE_TYPE_GATEWAY &&
        ping->hdr.device_type != DEVICE_TYPE_ANCHOR &&
        ping->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in PING_DEVICE from %d, ignoring", ping->hdr.device_type, dst_id);
        return;
    }

    switch (PRODUCT_DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            if (ping->hdr.device_type == DEVICE_TYPE_GATEWAY) {
                return;
            } else if (ping->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Update Infra storage with new RSSI and hop number
                update_infra_storage(dst_id, ping->hop_num, rssi_2);
            } else if (ping->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Update Sensor storage
                update_sensor_storage(dst_id, ping->version);
            }
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (ping->hdr.device_type == DEVICE_TYPE_GATEWAY) {
                // Set the mesh time
                mesh_time_set(ping->timestamp);
                // Check if we need to update hop number
                if (update_infra_storage(dst_id, ping->hop_num, rssi_2)) {
                    LOG_INF("Updated hop number for GATEWAY ID:%d to hop:%d based on PING_DEVICE", dst_id, PRODUCT_HOP_NUMBER);
                    device_updated_t pkt = {
                        .device_type = PRODUCT_DEVICE_TYPE,
                        .device_id = radio_get_device_id(),
                        .serial_num = PRODUCT_SERIAL_NUMBER,
                        .version = FIRMWARE_VERSION,
                        .connected_device_id = 0xFFFF,
                        .hop_num = PRODUCT_HOP_NUMBER,
                        .sensor_count = storage_sensor_count(),
                    };
                    send_device_updated(&pkt, dst_id, STATUS_SUCCESS);
                }
            } else if (ping->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if we need to update hop number
                if (update_infra_storage(dst_id, ping->hop_num, rssi_2)) {
                    LOG_INF("Updated hop number for ANCHOR ID:%d to hop:%d based on PING_DEVICE", dst_id, PRODUCT_HOP_NUMBER);
                    device_updated_t pkt = {
                        .device_type = PRODUCT_DEVICE_TYPE,
                        .device_id = radio_get_device_id(),
                        .serial_num = PRODUCT_SERIAL_NUMBER,
                        .version = FIRMWARE_VERSION,
                        .connected_device_id = 0xFFFF,
                        .hop_num = PRODUCT_HOP_NUMBER,
                        .sensor_count = storage_sensor_count(),
                    };
                    send_device_updated(&pkt, dst_id, STATUS_SUCCESS);
                }
                // Update mesh time if hop number is less than current hop number, which means the ping is from a closer anchor and the time is more accurate
                if (ping->hop_num < PRODUCT_HOP_NUMBER) {
                    mesh_time_set(ping->timestamp);
                    LOG_INF("Updated mesh time based on PING_DEVICE from ANCHOR ID:%d", dst_id);
                }
            } else if (ping->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Update mesh time
                mesh_time_set(ping->timestamp);
                update_sensor_storage(dst_id, ping->version);
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            if (ping->hdr.device_type == DEVICE_TYPE_GATEWAY || ping->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Update mesh time
                mesh_time_set(ping->timestamp);
                update_infra_storage(dst_id, ping->hop_num, rssi_2);
            }
        }
        break;

        default:
            LOG_WRN("Unknown device type 0x%02x in PING_DEVICE from %d, ignoring", ping->hdr.device_type, dst_id);
            return;
    }
    // Send ACK back to the sender
    send_ping_ack(dst_id, ping->hdr.tracking_id, STATUS_SUCCESS);

    LOG_INF("Mesh Time %llu seconds", mesh_time_get() / 1000);
    return;
}

/* Handle ping ack packet */
void handle_ping_ack(const ping_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const ping_ack_t *ping = (const ping_ack_t *)pkt;

    if (ping->hdr.device_id != radio_get_device_id()) {
        return;
    }

    // Remove the ping tracker
    tracker_remove_by_tracking_id(ping->hdr.tracking_id);

    LOG_INF("----> Received PING_ACK from %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(ping->hdr.device_type), dst_id, (rssi_2 / 2), ping->hdr.status);

    if (ping->hdr.device_type != DEVICE_TYPE_GATEWAY &&
        ping->hdr.device_type != DEVICE_TYPE_ANCHOR &&
        ping->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in PING_ACK from %d, ignoring", ping->hdr.device_type, dst_id);
        return;
    }

    switch (PRODUCT_DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            if (ping->hdr.device_type == DEVICE_TYPE_GATEWAY) {
                return;
            } else {
                // Print mesh time diff
                LOG_INF("Mesh time is %d seconds ahead based on PING_ACK from %s ID:%d", (int32_t)(mesh_time_get() - ping->timestamp), device_type_str(ping->hdr.device_type), dst_id);
                // Update Infra storage with new RSSI and hop number
                if (ping->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                    update_infra_storage(dst_id, ping->hop_num, rssi_2);
                } else if (ping->hdr.device_type == DEVICE_TYPE_SENSOR) {
                    update_sensor_storage(dst_id, ping->version);
                }
            }
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (ping->hdr.device_type == DEVICE_TYPE_GATEWAY) {
                // Update mesh time
                mesh_time_set(ping->timestamp);
                if (update_infra_storage(dst_id, ping->hop_num, rssi_2)) {
                    LOG_INF("Updated hop number for GATEWAY ID:%d to hop:%d based on PING_ACK", dst_id, PRODUCT_HOP_NUMBER);
                    device_updated_t pkt = {
                        .device_type = PRODUCT_DEVICE_TYPE,
                        .device_id = radio_get_device_id(),
                        .serial_num = PRODUCT_SERIAL_NUMBER,
                        .version = FIRMWARE_VERSION,
                        .connected_device_id = 0xFFFF,
                        .hop_num = PRODUCT_HOP_NUMBER,
                        .sensor_count = storage_sensor_count(),
                    };
                    send_device_updated(&pkt, dst_id, STATUS_SUCCESS);
                }
            } else if (ping->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if we need to update hop number
                if (update_infra_storage(dst_id, ping->hop_num, rssi_2)) {
                    LOG_INF("Updated hop number for ANCHOR ID:%d to hop:%d based on PING_ACK", dst_id, PRODUCT_HOP_NUMBER);
                    device_updated_t pkt = {
                        .device_type = PRODUCT_DEVICE_TYPE,
                        .device_id = radio_get_device_id(),
                        .serial_num = PRODUCT_SERIAL_NUMBER,
                        .version = FIRMWARE_VERSION,
                        .connected_device_id = 0xFFFF,
                        .hop_num = PRODUCT_HOP_NUMBER,
                        .sensor_count = storage_sensor_count(),
                    };
                    send_device_updated(&pkt, dst_id, STATUS_SUCCESS);
                }
                // Update mesh time if hop number is less than current hop number, which means the ping is from a closer anchor and the time is more accurate
                if (ping->hop_num < PRODUCT_HOP_NUMBER) {
                    mesh_time_set(ping->timestamp);
                    LOG_INF("Updated mesh time based on PING_DEVICE from ANCHOR ID:%d", dst_id);
                } else {
                    // Print mesh time diff
                    LOG_INF("Mesh time is %d seconds ahead based on PING_ACK from ANCHOR ID:%d", (int32_t)(mesh_time_get() - ping->timestamp), dst_id);
                }
            } else if (ping->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Print mesh time diff
                LOG_INF("Mesh time is %d seconds ahead based on PING_ACK from SENSOR ID:%d", (int32_t)(mesh_time_get() - ping->timestamp), dst_id);
                // Update Sensor storage
                update_sensor_storage(dst_id, ping->version);
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            if (ping->hdr.device_type == DEVICE_TYPE_GATEWAY || ping->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Update mesh time
                mesh_time_set(ping->timestamp);
                update_infra_storage(dst_id, ping->hop_num, rssi_2);
            }
        }
        break;

        default:
            LOG_WRN("Unknown device type 0x%02x in PING_ACK from %d, ignoring", ping->hdr.device_type, dst_id);
            return;
    }

    LOG_INF("Mesh Time %llu seconds", mesh_time_get() / 1000);
    return;
}

/* Handle device updated packet */
void handle_device_updated(const device_updated_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const device_updated_t *update = (const device_updated_t *)pkt;

    if (update->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("----> Received DEVICE_UPDATED from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(update->hdr.device_type), dst_id, device_type_str(update->device_id), update->device_id, (rssi_2 / 2), update->hdr.status);

    if (update->hdr.status != STATUS_SUCCESS) {
        return;
    }

    if (update->hdr.device_type != DEVICE_TYPE_GATEWAY &&
        update->hdr.device_type != DEVICE_TYPE_ANCHOR &&
        update->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in DEVICE_UPDATED from %d, ignoring", update->hdr.device_type, dst_id);
        return;
    }

    switch (PRODUCT_DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
            if (check_mesh_storage(update->device_id) == STATUS_ALREADY_EXISTS) {
                update_mesh_storage(update->device_id, update->hop_num, update->version, update->connected_device_id, update->sensor_count);
                LOG_INF("Updated mesh storage for device %s ID:%d based on DEVICE_UPDATED", device_type_str(update->hdr.device_type), update->device_id);
                send_device_updated_ack(update->device_id, dst_id, update->hdr.tracking_id, STATUS_SUCCESS);
            }
            break;
        
        case DEVICE_TYPE_ANCHOR:
            if (update->hdr.device_type == DEVICE_TYPE_ANCHOR || update->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // First ACK then upstream the update to gateway
                send_device_updated_ack(update->device_id, dst_id, update->hdr.tracking_id, STATUS_SUCCESS);
                
                LOG_INF("Forwarding DEVICE_UPDATED from %s ID:%d, updating infra storage", device_type_str(update->hdr.device_type), update->device_id);
                infra_entry_t entry;
                storage_infra_get(0, &entry);

                send_device_updated(update, entry.device_id, STATUS_SUCCESS);
            }
            break;

        case DEVICE_TYPE_SENSOR:
            LOG_WRN("Sensor will never receive DEVICE_UPDATED, ignoring");
            break;

        default:
            LOG_WRN("Unknown device type 0x%02x in DEVICE_UPDATED from %d, ignoring", update->hdr.device_type, dst_id);
            break;
    }
    return;
}

/* Handle device updated acknowledgment packet */
void handle_device_updated_ack(const device_updated_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const device_updated_ack_t *ack = (const device_updated_ack_t *)pkt;

    if (ack->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("----> Received DEVICE_UPDATED_ACK from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(ack->hdr.device_type), dst_id, device_type_str(ack->dst_device_id), ack->dst_device_id, (rssi_2 / 2), ack->hdr.status);

    /* Get the tracker entry to find prev_id (downstream route), then remove. */
    struct data_tracker *t = tracker_get_by_dst(ack->dst_device_id, PACKET_DEVICE_UPDATED);
    uint16_t prev_id = t ? t->prev_id : 0;
    tracker_remove_by_tracking_id(ack->hdr.tracking_id);

    switch (PRODUCT_DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
            return;

        case DEVICE_TYPE_ANCHOR:
            if (ack->dst_device_id != radio_get_device_id()) {
                LOG_INF("Forwarding DEVICE_UPDATED_ACK to device %s ID:%d: status 0x%02x", device_type_str(ack->dst_device_id), ack->dst_device_id, ack->hdr.status);
                send_device_updated_ack(ack->dst_device_id, prev_id, ack->hdr.tracking_id, ack->hdr.status);
            }
            return;

        case DEVICE_TYPE_SENSOR:
            return;

        default:
            LOG_WRN("Unknown device type 0x%02x in DEVICE_UPDATED_ACK from %d, ignoring", ack->hdr.device_type, dst_id);
            return;
    }
}

/* Handle repair request packet */
void handle_repair_request(const repair_request_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const repair_request_t *req = (const repair_request_t *)pkt;

    if (req->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("----> Received REPAIR_REQUEST from %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(req->hdr.device_type), dst_id, (rssi_2 / 2), req->hdr.status);

    /* Verify hash: hash should equal compute_pair_hash(dst_id, req->random_num) */
    uint32_t expected_hash = compute_pair_hash(dst_id, req->random_num);

    if (req->hash != expected_hash) {
        LOG_WRN("REPAIR_REQUEST hash mismatch from %s ID:%d (got 0x%08x, expected 0x%08x)", device_type_str(req->hdr.device_type), dst_id, req->hash, expected_hash);
        send_repair_response(dst_id, pkt->hdr.tracking_id, STATUS_AUTH_FAILED);
        return;
    }

    uint8_t status;
    switch(req->hdr.device_type) {
        case DEVICE_TYPE_GATEWAY:
            return;
        case DEVICE_TYPE_ANCHOR:
            // Check if device is already paired with this anchor
            if (check_infra_storage(dst_id, req->hdr.device_type, true) == STATUS_ALREADY_EXISTS) {
                status = STATUS_SUCCESS;
            } else {
                status = STATUS_NOT_FOUND;
            }
            break;
        case DEVICE_TYPE_SENSOR:
            // Check if device is already paired with this gateway/anchor
            if (check_sensor_storage(dst_id) == STATUS_ALREADY_EXISTS) {
                status = STATUS_SUCCESS;
            } else {
                status = STATUS_NOT_FOUND;
            }
            break;
        default:
            LOG_WRN("Unknown device type 0x%02x in REPAIR_REQUEST from %d, ignoring", req->hdr.device_type, dst_id);
            return;
    }

    send_repair_response(dst_id, pkt->hdr.tracking_id, status);
}

/* Handle repair response packet */
void handle_repair_response(const repair_response_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const repair_response_t *resp = (const repair_response_t *)pkt;

    if (resp->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("----> Received REPAIR_RESPONSE from %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(resp->hdr.device_type), dst_id, (rssi_2 / 2), resp->hdr.status);

    /* Remove the tracker. */
    tracker_remove_by_tracking_id(resp->hdr.tracking_id);

    if (resp->hdr.status == STATUS_SUCCESS) {
        switch(resp->hdr.device_type) {
            case DEVICE_TYPE_GATEWAY:
            case DEVICE_TYPE_ANCHOR:
            {
                switch(PRODUCT_DEVICE_TYPE) {
                    case DEVICE_TYPE_GATEWAY:
                        return;
                    
                    case DEVICE_TYPE_ANCHOR:
                        // Check if device is already paired with this anchor
                        if (check_infra_storage(dst_id, resp->hdr.device_type, true) == STATUS_ALREADY_EXISTS) {
                            if (update_infra_storage(dst_id, resp->hop_num, rssi_2)) {
                                LOG_INF("Updated infra storage for device %s ID:%d based on REPAIR_RESPONSE", device_type_str(resp->hdr.device_type), dst_id);
                            }
                        } else {
                            infra_entry_t entry;
                            entry.device_id = dst_id;
                            entry.device_type = resp->hdr.device_type;
                            entry.hop_num = resp->hop_num;
                            entry.rssi_2 = rssi_2;
                            entry.version = resp->version;
                            int err = storage_infra_add(&entry);
                            if (err) {
                                LOG_ERR("Failed to store repaired device %s ID:%d, err %d", device_type_str(resp->hdr.device_type), dst_id, err);
                                return;
                            }
                            LOG_INF("Stored repaired device %s ID:%d successfully", device_type_str(resp->hdr.device_type), dst_id);
                            product_info_update_hop();
                        }
                        break;

                    case DEVICE_TYPE_SENSOR:
                        // Check if device is already paired with this gateway/anchor
                        uint8_t status = check_infra_storage(dst_id, resp->hdr.device_type, false);
                        if (status == STATUS_ALREADY_EXISTS) {
                            break;
                        } else if (status != STATUS_STORAGE_FULL) {
                            infra_entry_t entry;
                            entry.device_id = dst_id;
                            entry.device_type = resp->hdr.device_type;
                            entry.hop_num = resp->hop_num;
                            entry.rssi_2 = rssi_2;
                            entry.version = resp->version;
                            storage_infra_add(&entry);
                            LOG_INF("Updated infra storage for repaired %s ID:%d based on REPAIR_RESPONSE", device_type_str(resp->hdr.device_type), dst_id);
                            // Testing purpose: send data init packet
                            update_known_devices();
                            sender.dst_id = dst_id;
                            data_init_t data_pkt = {
                                .gen_device_id = sender.gen_device_id,
                                .total_size = sender.total_size,
                                .chunk_count = sender.chunk_count,
                                .last_chunk_size = sender.last_chunk_size,
                                .crc32 = sender.crc32,
                            };
                            send_data_init(dst_id, sender.priority, &data_pkt);
                            return;
                        }
                        break;

                    default:
                        LOG_WRN("Unknown device type 0x%02x in REPAIR_RESPONSE from %d, ignoring", resp->hdr.device_type, dst_id);
                        return;
                }
                break;
            }
            case DEVICE_TYPE_SENSOR:
                return;
            default:
                LOG_WRN("Unknown device type 0x%02x in REPAIR_RESPONSE from %d, ignoring", resp->hdr.device_type, dst_id);
                return;
        }
        update_known_devices();
        
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

    LOG_INF("----> Received ROUTE_INFO from %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(route->hdr.device_type), dst_id, (rssi_2 / 2), route->hdr.status);

    switch (PRODUCT_DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            if (route->hdr.device_type == DEVICE_TYPE_ANCHOR || route->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Update the route info based on status
                if (route->hdr.status == STATUS_DEVICE_JOINED) {
                    err = storage_route_add(dst_id, route->device_id, route->device_type, route->route_len + 1, (route->avg_rssi_2 + rssi_2) / 2);
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
            }
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (route->hdr.device_type == DEVICE_TYPE_ANCHOR || route->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Update the route info based on status
                if (route->hdr.status == STATUS_DEVICE_JOINED) {
                    err = storage_route_add(dst_id, route->device_id, route->device_type, route->route_len + 1, (route->avg_rssi_2 + rssi_2) / 2);
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
                    infra_entry_t entry;
                    uint16_t next_hop_id[MAX_ANCHORS];
                    uint8_t next_hop_count = 0;

                    for (int i = 0; i < storage_infra_count(); i++) {
                        storage_infra_get(i, &entry);
                        if ((entry.device_type == DEVICE_TYPE_ANCHOR || entry.device_type == DEVICE_TYPE_GATEWAY) && entry.hop_num <= PRODUCT_HOP_NUMBER && entry.device_id != dst_id) {
                            next_hop_id[next_hop_count++] = entry.device_id;
                        }
                    }

                    route_info_t forward_pkt = {
                        .device_id = route->device_id,
                        .device_type = route->device_type,
                        .route_len = route->route_len + 1,
                        .avg_rssi_2 = (route->avg_rssi_2 + rssi_2) / 2,
                    };
                    for (int i = 0; i < next_hop_count; i++) {
                        LOG_INF("Forwarded ROUTE_INFO for device %s ID:%d to next hop %d", device_type_str(route->hdr.device_type), route->device_id, next_hop_id[i]);
                        send_route_info(&forward_pkt, next_hop_id[i], route->hdr.status);
                    }
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
    send_route_info_ack(&ack_pkt, dst_id, route->hdr.tracking_id, status);
    return;
}

/* Handle route info acknowledgment packet */
void handle_route_info_ack(const route_info_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const route_info_ack_t *ack = (const route_info_ack_t *)pkt;

    if (ack->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("----> Received ROUTE_INFO_ACK from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(ack->hdr.device_type), dst_id, device_type_str(ack->device_id), ack->device_id, (rssi_2 / 2), ack->hdr.status);

    /* Remove the tracker. */
    tracker_remove_by_tracking_id(ack->hdr.tracking_id);

    switch (PRODUCT_DEVICE_TYPE) {
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
                    send_route_info(&pkt, dst_id, ack->hdr.status);
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

    int available = MAX_ANCHORS - storage_infra_count();
    LOG_WRN("Infra storage has %d available slots for pairing and %d paired count", available, storage_infra_count());

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

        if (PRODUCT_HOP_NUMBER == 0xFF && c->hop_num == 0xFF) {
            LOG_WRN("Candidate %s ID:%d has no upstream link or has invalid hop number %d, skipping", device_type_str(c->device_type), c->sender_id, c->hop_num);
            continue;
        }

        send_pair_confirm(c->sender_id, STATUS_SUCCESS);

        if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_SENSOR) {
            // Sensor only pairs with one anchor, so stop after the first confirmation
            break;
        }
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
        LOG_INF("Collection window expired - %d candidates", resp_candidate_count);
        collecting = false;
        nbtimeout_stop(&collect_timer);
        select_and_confirm();
    }
}

/* Mesh time accessors. */

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