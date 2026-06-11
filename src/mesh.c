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
#include "config.h"
#include "mesh_layers/mesh_pairing.h"
#include "mesh_layers/mesh_session.h"
#include "log_color.h"

LOG_MODULE_REGISTER(mesh, CONFIG_MESH_LOG_LEVEL);

static bool mesh_initialized = false;
static uint16_t temp_id;
static uint64_t last_pair_request_sent_ms = 0;
static int64_t mesh_time_offset;

void mesh_init(void)
{
    if (mesh_initialized) {
        return;
    }

    temp_id = 0xFFFF;

    /* Gateway-only: anchor mesh_time at 0 for the moment of startup. */
    if (get_device_type() == DEVICE_TYPE_GATEWAY) {
        mesh_time_offset = -k_uptime_get();
    }

    mesh_initialized = true;
}

void mesh_tick(void)
{
    if (!mesh_initialized) {
        return;
    }

    mesh_pairing_tick();

    if (get_device_type() != DEVICE_TYPE_ANCHOR) {
        return;
    }

    for (int i = 0; i < infra_count; i++) {
        if (k_uptime_get() - infra_devices[i].last_comm_ms > (2 * PING_TIMEOUT_MS)) {
            // send ping device with status ping device to check if device is still reachable
            if (!infra_devices[i].is_ping_packet_sent) {
                send_ping_device(infra_devices[i].entry.device_id, infra_devices[i].entry.device_type, 0, STATUS_PING_DEVICE);
                infra_devices[i].is_ping_packet_sent = true;
            }
        }
    }

    if (infra_count < MAX_ANCHORS && (k_uptime_get() - last_pair_request_sent_ms > 3 * PING_TIMEOUT_MS)) {
        send_pair_request(false);
        last_pair_request_sent_ms = k_uptime_get();
    }
}

uint64_t mesh_time_get(void)
{
    return (uint64_t)(mesh_time_offset + k_uptime_get());
}

void mesh_time_set(uint64_t remote_mesh_time)
{
    mesh_time_offset = (int64_t)remote_mesh_time - k_uptime_get();
}

void set_temp_id(uint16_t id)
{
    temp_id = id;
}

uint16_t get_temp_id(void)
{
    return temp_id;
}

/* Check Infra Storage */
uint8_t check_infra_storage(uint16_t device_id, uint8_t device_type, bool is_it_sensor)
{
    // Check Only first slot as sensor can only pair with one gateway/anchor, but anchor can pair with multiple sensors
    if (is_it_sensor) {
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
bool update_infra_storage(uint16_t device_id, uint8_t hop_num, int16_t rssi_2)
{
    uint8_t current_hop_num = get_hop_number();
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
    if (current_hop_num != get_hop_number()) {
        return true;
    }
    return false;
}

/* Check Sensor Storage */
uint8_t check_sensor_storage(uint16_t device_id)
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
void update_sensor_storage(uint16_t device_id, uint16_t version)
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
uint8_t check_mesh_storage(uint16_t device_id)
{
    for (int idx = 0; idx < mesh_count; idx++) {
        if (mesh_devices[idx].device_id == device_id) {
            return STATUS_ALREADY_EXISTS;
        }
    }

    if (mesh_count >= MAX_DEVICES) {
        return STATUS_STORAGE_FULL;
    }

    return STATUS_SUCCESS;
}

/* Update Mesh Storage */
uint8_t update_mesh_storage(uint16_t device_id, uint8_t hop_num, uint16_t version, uint16_t connected_device_id, uint8_t sensor_cnt)
{
    mesh_entry_t entry;
    int idx = -1;

    for (idx = 0; idx < mesh_count; idx++) {
        if (mesh_devices[idx].device_id == device_id) {
            break;
        }
        if (idx == mesh_count - 1) {
            LOG_WRN("Device ID %d not found in mesh storage", device_id);
            return STATUS_NOT_FOUND;
        }
    }

    if (storage_mesh_get(idx, &entry) == 0) {
        if (entry.hop_num != hop_num && hop_num != 0xFF) {
            entry.hop_num = hop_num;
            mesh_devices[idx].hop_num = hop_num;
        }
        if (entry.version != version) {
            entry.version = version;
        }
        if (entry.connected_device_id != connected_device_id && connected_device_id != 0xFFFF) {
            entry.connected_device_id = connected_device_id;
            mesh_devices[idx].connected_device_id = connected_device_id;
        }
        if (entry.sensor_count != sensor_cnt && sensor_cnt != 0xFF) {
            entry.sensor_count = sensor_cnt;
        }
        int err = storage_mesh_update(idx, &entry);
        if (err) {
            LOG_ERR("Failed to update mesh entry for device %d, err %d", device_id, err);
            return STATUS_FAILURE;
        }
    } else {
        LOG_ERR("Failed to get mesh entry for device %d, err %d", device_id, idx);
        return STATUS_FAILURE;
    }
    return STATUS_SUCCESS;
}
