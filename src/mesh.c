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

/* Send pairing request packet. */
int send_pair_request(uint32_t handle, uint8_t tracking_id)
{
    uint32_t random_num = generate_random_number();
    uint32_t hash = compute_pair_hash(radio_get_device_id(), random_num);

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
        .hash = hash,
    };

    /* Store packet as tracker payload for retries. */
    tracker_update_payload(tracking_id, &packet, sizeof(packet));

    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send pairing response packet. */
int send_pair_response(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status, uint8_t hop_num)
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

    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
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
        .version = FIRMWARE_VERSION,
    };

    /* Store packet as tracker payload for retries. */
    tracker_update_payload(tracking_id, &packet, sizeof(packet));

    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
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

    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
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

    // Update tracker payload for retries in case of mesh packet loss
    tracker_update_payload(tracking_id, &packet, sizeof(packet));

    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send joined network acknowledgment packet. */
int send_joined_network_ack(uint32_t handle, uint16_t dst_device_id, uint16_t dst_id, uint8_t tracking_id, uint8_t status)
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

    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send ping device packet. */
int send_ping_device(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status)
{
    ping_device_t packet = {
        .hdr = {
            .packet_type = PACKET_PING_DEVICE,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_LOW,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .hop_num = PRODUCT_HOP_NUMBER,
        .version = FIRMWARE_VERSION,
    };

    // Update tracker payload for retries in case of mesh packet loss
    tracker_update_payload(tracking_id, &packet, sizeof(packet));

    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send ping acknowledgment packet. */
int send_ping_ack(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status)
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
    };

    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send device updated packet. */
int send_device_updated(uint32_t handle, const device_updated_t *pkt, uint16_t dst_id, uint8_t tracking_id, uint8_t status)
{
    device_updated_t packet = {
        .hdr = {
            .packet_type = PACKET_DEVICE_UPDATED,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_LOW,
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

    // Update tracker payload for retries in case of mesh packet loss
    tracker_update_payload(tracking_id, &packet, sizeof(packet));

    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send device updated acknowledgment packet. */
int send_device_updated_ack(uint32_t handle, uint16_t dst_device_id, uint16_t dst_id, uint8_t tracking_id, uint8_t status)
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

    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send repair request packet. */
int send_repair_request(uint32_t handle, uint16_t dst_id, uint8_t tracking_id)
{
    uint32_t random_num = generate_random_number();
    uint32_t hash = compute_pair_hash(radio_get_device_id(), random_num);

    repair_request_t packet = {
        .hdr = {
            .packet_type = PACKET_REPAIR_REQUEST,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = STATUS_SUCCESS,
        },
        .random_num = random_num,
        .hash = hash,
        .version = FIRMWARE_VERSION,
        .hop_num = PRODUCT_HOP_NUMBER,
    };

    // Update tracker payload for retries in case of mesh packet loss
    tracker_update_payload(tracking_id, &packet, sizeof(packet));

    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send repair response packet. */
int send_repair_response(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status)
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

    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send SYNC_TIME packet — pass dst_id = 0 to broadcast. */
int send_sync_time(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status)
{
    sync_time_t packet = {
        .hdr = {
            .packet_type = PACKET_SYNC_TIME,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_LOW,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .timestamp = mesh_time_get() + 15,
    };

    // Update tracker payload for retries in case of mesh packet loss
    tracker_update_payload(tracking_id, &packet, sizeof(packet));

    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send SYNC_TIME_ACK back to the sender of a SYNC_TIME. */
int send_sync_time_ack(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status)
{
    sync_time_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_SYNC_TIME_ACK,
            .device_type = PRODUCT_DEVICE_TYPE,
            .priority = PACKET_PRIORITY_LOW,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .timestamp = mesh_time_get(),
    };

    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Handle received pairing request packet. */
void handle_pair_request(const pair_request_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    LOG_INF("PAIR_REQUEST from %s ID:%d and RSSI:%d", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2));

    /* Verify hash: hash should equal compute_pair_hash(sender_id, random_num). */
    uint32_t expected_hash = compute_pair_hash(dst_id, pkt->random_num);

    if (pkt->hash != expected_hash) {
        LOG_WRN("PAIR_REQUEST hash mismatch from %d (got 0x%08x, expected 0x%08x)",
            dst_id, pkt->hash, expected_hash);
        send_pair_response(0, dst_id, pkt->hdr.tracking_id, STATUS_AUTH_FAILED, 0);
        return;
    }

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
            // Testing only
            if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_GATEWAY) {
                return;
            }
            break;
        default:
            LOG_WRN("Unknown device type 0x%02x in pair request from %d, rejecting", pkt->hdr.device_type, dst_id);
            return;
    }

    LOG_INF("Sending PAIR_RESPONSE to %d: status 0x%02x", dst_id, status);
    send_pair_response(0, dst_id, pkt->hdr.tracking_id, status, PRODUCT_HOP_NUMBER);
    if (status == STATUS_ALREADY_EXISTS) {
        // send sync time packet
        uint8_t tid = tracker_next_id();
        tracker_add(radio_get_device_id(), dst_id, tid, PACKET_SYNC_TIME, 500, 5, NULL, 0);
        LOG_INF("Sending SYNC_TIME to %d since it's already paired", dst_id);
        send_sync_time(0, dst_id, tid, STATUS_SUCCESS);
    }
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

    /* Stop the request tracker — pair request was broadcast (device_id = 0). */
    tracker_remove_by_dst(radio_get_device_id(), PACKET_PAIR_REQUEST);

    if (resp->hdr.status != STATUS_SUCCESS && resp->hdr.status != STATUS_ALREADY_EXISTS) {
        LOG_WRN("PAIR_RESPONSE failed: status 0x%02x", resp->hdr.status);
        return;
    }

    if (resp->hdr.status == STATUS_ALREADY_EXISTS) {
        // Check if we already have this device in storage
        if (resp->hdr.device_type == DEVICE_TYPE_SENSOR) {
            if (check_sensor_storage(dst_id) == STATUS_SUCCESS) {
                // send repair request
                uint8_t tid = tracker_next_id();
                tracker_add(radio_get_device_id(), dst_id, tid, PACKET_REPAIR_REQUEST, 500, 5, NULL, 0);
                LOG_INF("Sending REPAIR_REQUEST to SENSOR ID:%d since it's already paired but not in storage", dst_id);
                send_repair_request(0, dst_id, tid);
            }
        } else {
            if (check_infra_storage(dst_id, resp->hdr.device_type, true) == STATUS_SUCCESS) {
                // send repair request
                uint8_t tid = tracker_next_id();
                tracker_add(radio_get_device_id(), dst_id, tid, PACKET_REPAIR_REQUEST, 500, 5, NULL, 0);
                LOG_INF("Sending REPAIR_REQUEST to %s ID:%d since it's already paired but not in storage", device_type_str(resp->hdr.device_type), dst_id);
                send_repair_request(0, dst_id, tid);
            }
        }
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
    if((conf->hdr.device_type != DEVICE_TYPE_SENSOR) && (conf->hdr.device_type != DEVICE_TYPE_ANCHOR)) {
        LOG_WRN("PAIR_CONFIRM from unsupported device %s ID:%d: OR status 0x%02x", device_type_str(conf->hdr.device_type), dst_id, conf->hdr.status);
        return;
    } else if (conf->hdr.status != STATUS_SUCCESS) {
        LOG_WRN("PAIR_CONFIRM failed from device %s ID:%d: status 0x%02x", device_type_str(conf->hdr.device_type), dst_id, conf->hdr.status);
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
                entry.version = conf->version;
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
                entry.version = conf->version;
                int err = storage_sensor_add(&entry);
                if (err) {
                    LOG_ERR("Failed to store paired sensor, err %d", err);
                    return;
                }
                LOG_INF("Sensor %d paired and stored (total %d)", dst_id, storage_sensor_count());
                radio_update_known_devices();

                if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_ANCHOR) {
                    // Send Device Updated Packet to Gateway about new paired sensor
                    uint8_t tid = tracker_next_id();
                    tracker_add(radio_get_device_id(), radio_get_device_id(), tid, PACKET_DEVICE_UPDATED, PRODUCT_HOP_NUMBER * 500, 5, NULL, 0);
                    
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
                    LOG_INF("Sending DEVICE_UPDATED to gateway about new paired SENSOR %d", dst_id);
                    send_device_updated(0, &pkt, entry.device_id, tid, STATUS_SUCCESS);
                }
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
    // Send SYNC_TIME packet if already paired to sync time with the new device
    if (status == STATUS_SUCCESS || status == STATUS_ALREADY_EXISTS) {
        uint8_t tid = tracker_next_id();
        tracker_add(radio_get_device_id(), dst_id, tid, PACKET_SYNC_TIME, 500, 5, NULL, 0);
        LOG_INF("Sending SYNC_TIME to %d since it's already paired", dst_id);
        send_sync_time(0, dst_id, tid, STATUS_SUCCESS);
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

    LOG_INF("PAIR_ACK from %s ID:%d: status 0x%02x", device_type_str(ack->hdr.device_type), dst_id, ack->hdr.status);

    /* Remove the confirm tracker. */
    tracker_remove_by_dst(radio_get_device_id(), PACKET_PAIR_CONFIRM);

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
        entry.version = ack->version;
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

        if (storage_infra_count() == 1) {
            uint8_t tid = tracker_next_id();
            tracker_add(radio_get_device_id(), radio_get_device_id(), tid, PACKET_JOINED_NETWORK, (ack->hop_num + 1) * 500, 5, &jn_pkt, sizeof(jn_pkt));

            LOG_INF("Sending JOINED_NETWORK to %d for %s: hop %d", dst_id, device_type_str(ack->hdr.device_type), jn_pkt.hop_num);
            send_joined_network(0, &jn_pkt, dst_id, tid, STATUS_SUCCESS);
        }
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
            case DEVICE_TYPE_SENSOR:
                // Upstream the Packet to Gateway if Anchor receives the JOINED_NETWORK packet from Sensor
                if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_ANCHOR) {
                    LOG_INF("Forwarding JOINED_NETWORK from %s %d upstream", device_type_str(jn->device_type), dst_id);
                    infra_entry_t entry;
                    storage_infra_get(0, &entry);
                    
                    uint8_t tid = tracker_next_id();
                    tracker_add(jn->device_id, dst_id, tid, PACKET_JOINED_NETWORK, PRODUCT_HOP_NUMBER * 500, 5, jn, sizeof(*jn));
                    send_joined_network(0, jn, entry.device_id, tid, STATUS_SUCCESS);
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
                            send_joined_network_ack(0, jn->device_id, dst_id, jn->hdr.tracking_id, STATUS_FAILURE);
                            return;
                        }
                        LOG_INF("%s %d successfully joined network", device_type_str(jn->device_type), jn->device_id);
                        LOG_INF("Sending JOINED_NETWORK_ACK to %d for device %d", dst_id, jn->device_id);
                    }
                    send_joined_network_ack(0, jn->device_id, dst_id, jn->hdr.tracking_id, status);
                } else if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_SENSOR) {
                    LOG_WRN("Sensor will never receive JOINED_NETWORK from anchor, ignoring");
                    return;
                }
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

    /* Get the tracker entry to find prev_id (downstream route), then remove. */
    struct data_tracker *t = tracker_get_by_dst(ack->dst_device_id, PACKET_JOINED_NETWORK);
    uint16_t prev_id = t ? t->prev_id : 0;
    tracker_remove_by_dst(ack->dst_device_id, PACKET_JOINED_NETWORK);

    switch (PRODUCT_DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
            LOG_WRN("Gateway will never receive JOINED_NETWORK_ACK, ignoring");
            return;
        
        case DEVICE_TYPE_ANCHOR:
            if (ack->dst_device_id == radio_get_device_id()) {
                LOG_INF("Received JOINED_NETWORK_ACK from GATEWAY: status 0x%02x", ack->hdr.status);
            } else {
                LOG_INF("Forwarding JOINED_NETWORK_ACK to device %d: status 0x%02x", ack->dst_device_id, ack->hdr.status);
                send_joined_network_ack(0, ack->dst_device_id, prev_id, ack->hdr.tracking_id, ack->hdr.status);
            }
            return;

        case DEVICE_TYPE_SENSOR:
            LOG_INF("Received JOINED_NETWORK_ACK from %d for device %d: status 0x%02x", dst_id, ack->dst_device_id, ack->hdr.status);
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

    LOG_INF("PING_DEVICE from %s ID:%d: status 0x%02x", device_type_str(ping->hdr.device_type), dst_id, ping->hdr.status);

    if (ping->hdr.device_type != DEVICE_TYPE_GATEWAY &&
        ping->hdr.device_type != DEVICE_TYPE_ANCHOR &&
        ping->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("PING_DEVICE from unknown device type 0x%02x, ignoring", ping->hdr.device_type);
        return;
    }

    if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_GATEWAY || PRODUCT_DEVICE_TYPE == DEVICE_TYPE_ANCHOR) {
        // Respond with PING_ACK
        LOG_INF("Sending PING_ACK to %d: status 0x%02x", dst_id, STATUS_SUCCESS);
        send_ping_ack(0, dst_id, ping->hdr.tracking_id, STATUS_SUCCESS);
        if (update_infra_storage(dst_id, ping->hop_num, rssi_2) && PRODUCT_DEVICE_TYPE == DEVICE_TYPE_ANCHOR) {
            LOG_INF("Sending DEVICE_UPDATED to %d for %s with new hop number %d", dst_id, device_type_str(ping->hdr.device_type), PRODUCT_HOP_NUMBER);
            uint8_t tid = tracker_next_id();
            tracker_add(radio_get_device_id(), radio_get_device_id(), tid, PACKET_DEVICE_UPDATED, PRODUCT_HOP_NUMBER * 500, 5, NULL, 0);
            device_updated_t pkt = {
                .device_type = PRODUCT_DEVICE_TYPE,
                .device_id = radio_get_device_id(),
                .serial_num = PRODUCT_SERIAL_NUMBER,
                .version = FIRMWARE_VERSION,
                .connected_device_id = 0xFFFF,
                .hop_num = PRODUCT_HOP_NUMBER,
                .sensor_count = storage_sensor_count(),
            };
            send_device_updated(0, &pkt, dst_id, tid, STATUS_SUCCESS);
        }
        return;
    }
}

/* Handle ping ack packet */
void handle_ping_ack(const ping_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const ping_ack_t *ping = (const ping_ack_t *)pkt;

    if (ping->hdr.device_id != radio_get_device_id()) {
        return;
    }

    // Remove the ping tracker
    tracker_remove_by_dst(radio_get_device_id(), PACKET_PING_DEVICE);

    LOG_INF("PING_ACK from %s ID:%d: status 0x%02x", device_type_str(ping->hdr.device_type), dst_id, ping->hdr.status);

    if (ping->hdr.device_type != DEVICE_TYPE_GATEWAY &&
        ping->hdr.device_type != DEVICE_TYPE_ANCHOR &&
        ping->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("PING_ACK from unknown device type 0x%02x, ignoring", ping->hdr.device_type);
        return;
    }

    if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_GATEWAY || PRODUCT_DEVICE_TYPE == DEVICE_TYPE_ANCHOR) {
        if (update_infra_storage(dst_id, ping->hop_num, rssi_2) && PRODUCT_DEVICE_TYPE == DEVICE_TYPE_ANCHOR) {
            LOG_INF("Updated hop number for device %d to %d based on PING_ACK", dst_id, PRODUCT_HOP_NUMBER);
            uint8_t tid = tracker_next_id();
            tracker_add(radio_get_device_id(), radio_get_device_id(), tid, PACKET_DEVICE_UPDATED, PRODUCT_HOP_NUMBER * 500, 5, NULL, 0);
            device_updated_t pkt = {
                .device_type = PRODUCT_DEVICE_TYPE,
                .device_id = radio_get_device_id(),
                .serial_num = PRODUCT_SERIAL_NUMBER,
                .version = FIRMWARE_VERSION,
                .connected_device_id = 0xFFFF,
                .hop_num = PRODUCT_HOP_NUMBER,
                .sensor_count = storage_sensor_count(),
            };
            send_device_updated(0, &pkt, dst_id, tid, STATUS_SUCCESS);
        }
        return;
    }
}

/* Handle device updated packet */
void handle_device_updated(const device_updated_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const device_updated_t *update = (const device_updated_t *)pkt;

    if (update->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("DEVICE_UPDATED from %s ID:%d: status 0x%02x", device_type_str(update->hdr.device_type), dst_id, update->hdr.status);

    if (update->hdr.status != STATUS_SUCCESS) {
        LOG_WRN("DEVICE_UPDATED failed with status 0x%02x", update->hdr.status);
        return;
    }

    if (update->hdr.device_type != DEVICE_TYPE_GATEWAY &&
        update->hdr.device_type != DEVICE_TYPE_ANCHOR &&
        update->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("DEVICE_UPDATED from unknown device type 0x%02x, ignoring", update->hdr.device_type);
        return;
    }

    switch (PRODUCT_DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
            if (check_mesh_storage(update->device_id) == STATUS_ALREADY_EXISTS) {
                update_mesh_storage(update->device_id, update->hop_num, update->version, update->connected_device_id, update->sensor_count);
                LOG_INF("Updated mesh storage for device %s: %d based on DEVICE_UPDATED", device_type_str(update->hdr.device_type), update->device_id);
                LOG_INF("Sending DEVICE_UPDATED_ACK to %d for device %d", dst_id, update->device_id);
                send_device_updated_ack(0, update->device_id, dst_id, update->hdr.tracking_id, STATUS_SUCCESS);
            }
            break;
        
        case DEVICE_TYPE_ANCHOR:
            if (update->hdr.device_type == DEVICE_TYPE_ANCHOR || update->hdr.device_type == DEVICE_TYPE_SENSOR) {
                LOG_INF("Forwarding DEVICE_UPDATED from %s: %d, updating infra storage", device_type_str(update->hdr.device_type), update->device_id);
                infra_entry_t entry;
                storage_infra_get(0, &entry);

                uint8_t tid = tracker_next_id();
                tracker_add(radio_get_device_id(), dst_id, tid, PACKET_DEVICE_UPDATED, PRODUCT_HOP_NUMBER * 500, 5, NULL, 0);
                send_device_updated(0, update, entry.device_id, tid, STATUS_SUCCESS);
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

    LOG_INF("DEVICE_UPDATED_ACK from %s ID:%d: status 0x%02x", device_type_str(ack->hdr.device_type), dst_id, ack->hdr.status);

    /* Get the tracker entry to find prev_id (downstream route), then remove. */
    struct data_tracker *t = tracker_get_by_dst(ack->dst_device_id, PACKET_DEVICE_UPDATED);
    uint16_t prev_id = t ? t->prev_id : 0;
    tracker_remove_by_dst(ack->dst_device_id, PACKET_DEVICE_UPDATED);

    switch (PRODUCT_DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
            LOG_WRN("Gateway will never receive DEVICE_UPDATED_ACK, ignoring");
            return;

        case DEVICE_TYPE_ANCHOR:
            if (ack->dst_device_id == radio_get_device_id()) {
                LOG_INF("Received DEVICE_UPDATED_ACK from GATEWAY: status 0x%02x", ack->hdr.status);
            } else {
                LOG_INF("Forwarding DEVICE_UPDATED_ACK to device %d: status 0x%02x", ack->dst_device_id, ack->hdr.status);
                send_device_updated_ack(0, ack->dst_device_id, prev_id, ack->hdr.tracking_id, ack->hdr.status);
            }
            return;

        case DEVICE_TYPE_SENSOR:
            LOG_INF("Received DEVICE_UPDATED_ACK from %d for device %d: status 0x%02x", dst_id, ack->dst_device_id, ack->hdr.status);
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

    LOG_INF("Received REPAIR_REQUEST from %s ID:%d", device_type_str(req->hdr.device_type), dst_id);

    /* Verify hash: hash should equal compute_pair_hash(dst_id, req->random_num) */
    uint32_t expected_hash = compute_pair_hash(dst_id, req->random_num);

    if (req->hash != expected_hash) {
        LOG_WRN("REPAIR_REQUEST hash mismatch from %d (got 0x%08x, expected 0x%08x)",
            dst_id, req->hash, expected_hash);
        send_repair_response(0, dst_id, pkt->hdr.tracking_id, STATUS_AUTH_FAILED);
        return;
    }

    uint8_t status;
    switch(req->hdr.device_type) {
        case DEVICE_TYPE_GATEWAY:
            LOG_WRN("Received REPAIR_REQUEST from gateway device %d, ignoring", dst_id);
            return;
        case DEVICE_TYPE_ANCHOR:
            // Check if device is already paired with this anchor
            if (check_infra_storage(dst_id, req->hdr.device_type, true) == STATUS_ALREADY_EXISTS) {
                LOG_INF("Repair request from known %s ID:%d, sending success response", device_type_str(req->hdr.device_type), dst_id);
                status = STATUS_SUCCESS;
            } else {
                LOG_WRN("Repair request from unknown %s ID:%d, sending failure response", device_type_str(req->hdr.device_type), dst_id);
                status = STATUS_NOT_FOUND;
            }
            break;
        case DEVICE_TYPE_SENSOR:
            // Check if device is already paired with this gateway/anchor
            if (check_sensor_storage(dst_id) == STATUS_ALREADY_EXISTS) {
                LOG_INF("Repair request from known %s ID:%d, sending success response", device_type_str(req->hdr.device_type), dst_id);
                status = STATUS_SUCCESS;
            } else {
                LOG_WRN("Repair request from unknown %s ID:%d, sending failure response", device_type_str(req->hdr.device_type), dst_id);
                status = STATUS_NOT_FOUND;
            }
            break;
        default:
            LOG_WRN("Unknown device type 0x%02x in REPAIR_REQUEST from %d, ignoring", req->hdr.device_type, dst_id);
            return;
    }

    LOG_INF("Sending REPAIR_RESPONSE to %s ID:%d status 0x%02x", device_type_str(pkt->hdr.device_type), dst_id, status);
    send_repair_response(0, dst_id, pkt->hdr.tracking_id, status);
}

/* Handle repair response packet */
void handle_repair_response(const repair_response_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    const repair_response_t *resp = (const repair_response_t *)pkt;

    if (resp->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("Received REPAIR_RESPONSE from %s ID:%d: status 0x%02x", device_type_str(resp->hdr.device_type), dst_id, resp->hdr.status);

    /* Remove the tracker. */
    tracker_remove_by_dst(radio_get_device_id(), PACKET_REPAIR_REQUEST);

    if (resp->hdr.status == STATUS_SUCCESS) {
        switch(resp->hdr.device_type) {
            case DEVICE_TYPE_GATEWAY:
            case DEVICE_TYPE_ANCHOR:
            {
                switch(PRODUCT_DEVICE_TYPE) {
                    case DEVICE_TYPE_GATEWAY:
                        LOG_WRN("Gateway will never receive REPAIR_RESPONSE, ignoring");
                        return;
                    
                    case DEVICE_TYPE_ANCHOR:
                        // Check if device is already paired with this anchor
                        if (check_infra_storage(dst_id, resp->hdr.device_type, true) == STATUS_ALREADY_EXISTS) {
                            if (update_infra_storage(dst_id, resp->hop_num, rssi_2)) {
                                LOG_INF("Updated infra storage for device %d based on REPAIR_RESPONSE", dst_id);
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
                                LOG_ERR("Failed to store repaired device, err %d", err);
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
                            LOG_INF("Received REPAIR_RESPONSE with success from known %s ID:%d, but no storage update needed since sensors don't store hop/RSSI", device_type_str(resp->hdr.device_type), dst_id);
                        } else if (status != STATUS_STORAGE_FULL) {
                            infra_entry_t entry;
                            entry.device_id = dst_id;
                            entry.device_type = resp->hdr.device_type;
                            entry.hop_num = resp->hop_num;
                            entry.rssi_2 = rssi_2;
                            entry.version = resp->version;
                            storage_infra_add(&entry);
                            LOG_INF("Updated infra storage for repaired %s ID:%d based on REPAIR_RESPONSE", device_type_str(resp->hdr.device_type), dst_id);
                        }
                        break;

                    default:
                        LOG_WRN("Unknown device type 0x%02x in REPAIR_RESPONSE from %d, ignoring", resp->hdr.device_type, dst_id);
                        return;
                }
                break;
            }
            case DEVICE_TYPE_SENSOR:
                LOG_WRN("Received REPAIR_RESPONSE from sensor device %d, ignoring", dst_id);
                return;
            default:
                LOG_WRN("Unknown device type 0x%02x in REPAIR_RESPONSE from %d, ignoring", resp->hdr.device_type, dst_id);
                return;
        }
        radio_update_known_devices();
        
    } else {
        LOG_WRN("REPAIR_RESPONSE failed with status 0x%02x", resp->hdr.status);
    }
}

/* Handle SYNC_TIME packet. */
void handle_sync_time(const sync_time_t *sync, uint16_t sender_id, int16_t rssi_2)
{
    const sync_time_t *pkt = (const sync_time_t *)sync;

    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    LOG_INF("SYNC_TIME from %s ID:%d: timestamp %llu", device_type_str(pkt->hdr.device_type), sender_id, pkt->timestamp);
    
    switch (pkt->hdr.device_type) {
        case DEVICE_TYPE_GATEWAY:
        case DEVICE_TYPE_ANCHOR:
            mesh_time_set(pkt->timestamp);
            LOG_INF("Mesh time updated to %llu based on SYNC_TIME from %s ID:%d", pkt->timestamp, device_type_str(pkt->hdr.device_type), sender_id);
            break;

        
        case DEVICE_TYPE_SENSOR:
            LOG_WRN("Received SYNC_TIME from sensor device %d, ignoring", sender_id);
            return;

        default:
            LOG_WRN("Unknown device type 0x%02x in SYNC_TIME from %d, ignoring", pkt->hdr.device_type, sender_id);
            return;
    }

    LOG_INF("Sending SYNC_TIME_ACK to %d: status 0x%02x", sender_id, STATUS_SUCCESS);
    send_sync_time_ack(0, sender_id, pkt->hdr.tracking_id, STATUS_SUCCESS);

}

/* Handle SYNC_TIME_ACK packet. */
void handle_sync_time_ack(const sync_time_ack_t *ack, uint16_t sender_id, int16_t rssi_2)
{
    const sync_time_ack_t *pkt = (const sync_time_ack_t *)ack;

    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    // Remove the tracker
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);
    tracker_remove_by_dst(radio_get_device_id(), PACKET_SYNC_TIME);

    LOG_INF("SYNC_TIME_ACK from %s ID:%d: status 0x%02x", device_type_str(pkt->hdr.device_type), sender_id, pkt->hdr.status);

    switch (pkt->hdr.device_type) {
        case DEVICE_TYPE_GATEWAY:
            LOG_WRN("Received SYNC_TIME_ACK from gateway device %d, ignoring", sender_id);
            return;

        case DEVICE_TYPE_ANCHOR:
        case DEVICE_TYPE_SENSOR:
            LOG_INF("Received SYNC_TIME_ACK from %s ID:%d, difference is %lld ms ", device_type_str(pkt->hdr.device_type), sender_id, (int64_t)mesh_time_get() - (int64_t)pkt->timestamp);
            break;

        default:
            LOG_WRN("Unknown device type 0x%02x in SYNC_TIME_ACK from %d, ignoring", pkt->hdr.device_type, sender_id);
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

    /* How many slots available?
     * Sensor: only 1 upstream connection allowed.
     * Anchor: up to MAX_ANCHORS. */
    int max_slots = (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_SENSOR) ? 1 : MAX_ANCHORS;
    int available = max_slots - storage_infra_count();
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

        uint8_t tid = tracker_next_id();

        LOG_INF("Sending PAIR_CONFIRM to %s ID:%d (hop:%d, RSSI:%d.%d, tid:%d)",
            device_type_str(c->device_type), c->sender_id,
            c->hop_num, (c->rssi_2 / 2), (c->rssi_2 & 0b1) * 5, tid);

        tracker_add(radio_get_device_id(), c->sender_id, tid, PACKET_PAIR_CONFIRM, PAIR_TIMEOUT_MS, PAIR_MAX_RETRIES, NULL, 0);
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

#define MESH_TIME_MILESTONE_MS (1ULL * 60ULL * 1000ULL)

void mesh_time_check_milestone(void)
{
    static uint64_t last_logged_bucket;

    uint64_t now = mesh_time_get();
    uint64_t bucket = now / MESH_TIME_MILESTONE_MS;

    if (bucket > 0 && bucket != last_logged_bucket) {
        last_logged_bucket = bucket;
        LOG_INF("Mesh time milestone: %llu min",
            bucket * (MESH_TIME_MILESTONE_MS / 60000));
    }
}