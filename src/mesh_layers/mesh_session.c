

#include <string.h>
#include <zephyr/kernel.h>
#include "mesh_session.h"
#include "mesh_pairing.h"
#include "mesh_routing.h"
#include "../mesh.h"
#include "../protocol.h"
#include "../tracker.h"
#include "../queue.h"
#include "../product_info.h"
#include "../log_color.h"
#include "../main_sub.h"
#include "../radio.h"
#include "../storage.h"

LOG_MODULE_REGISTER(mesh_session, CONFIG_MESH_SESSION_LOG_LEVEL);

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------------**** TX Helpers ****------------------------------------------------------------------------------- */

/* Send joined network packet. */
int send_joined_network(const joined_network_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t status)
{
    joined_network_t packet = {
        .hdr = {
            .packet_type = PACKET_JOINED_NETWORK,
            .device_type = get_device_type(),
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
    tracker_add(dst_id, get_device_id(), packet.hdr.tracking_id, PACKET_JOINED_NETWORK, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF_GRN("Sending JOINED_NETWORK to device %s ID:%d for device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, device_type_str(pkt->device_type), pkt->device_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send joined network acknowledgment packet. */
int send_joined_network_ack(const joined_network_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status)
{
    joined_network_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_JOINED_NETWORK_ACK,
            .device_type = get_device_type(),
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .dst_device_id = pkt->dst_device_id,
        .dst_device_type = pkt->dst_device_type,
    };

    LOG_INF_GRN("Sending JOINED_NETWORK_ACK to device %s ID:%d for device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, device_type_str(pkt->dst_device_type), pkt->dst_device_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send ping device packet. */
int send_ping_device(uint16_t dst_id, uint8_t dst_type, uint16_t gen_id, uint8_t status)
{
    ping_device_t packet = {
        .hdr = {
            .packet_type = PACKET_PING_DEVICE,
            .device_type = get_device_type(),
            .priority = PACKET_PRIORITY_LOW,
            .tracking_id = tracker_next_id(),
            .device_id = dst_id,
            .status = status,
        },
        .dst_device_id = gen_id,
        .hop_num = get_hop_number(),
        .version = FIRMWARE_VERSION,
        .total_devices = get_mesh_devices_count(),
        .timestamp = mesh_time_get() + 15,
    };

    // Add tracker entry for retries
    tracker_add(dst_id, get_device_id(), packet.hdr.tracking_id, PACKET_PING_DEVICE, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF_GRN("Sending PING_DEVICE to device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send ping acknowledgment packet. */
int send_ping_ack(uint16_t dst_id, uint8_t dst_type, uint16_t gen_id, uint8_t tracking_id, uint8_t status)
{
    ping_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_PING_ACK,
            .device_type = get_device_type(),
            .priority = PACKET_PRIORITY_LOW,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .dst_device_id = gen_id,
        .hop_num = get_hop_number(),
        .version = FIRMWARE_VERSION,
        .timestamp = mesh_time_get() + 15,
    };

    LOG_INF_GRN("Sending PING_ACK to device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send device updated packet. */
int send_device_updated(const device_updated_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t status)
{
    device_updated_t packet = {
        .hdr = {
            .packet_type = PACKET_DEVICE_UPDATED,
            .device_type = get_device_type(),
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
    tracker_add(dst_id, get_device_id(), packet.hdr.tracking_id, PACKET_DEVICE_UPDATED, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF_GRN("Sending DEVICE_UPDATED to device %s ID:%d for device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, device_type_str(pkt->device_type), pkt->device_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send device updated acknowledgment packet. */
int send_device_updated_ack(const device_updated_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status)
{
    device_updated_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_DEVICE_UPDATED_ACK,
            .device_type = get_device_type(),
            .priority = PACKET_PRIORITY_LOW,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .dst_device_id = pkt->dst_device_id,
        .dst_device_type = pkt->dst_device_type,
    };

    LOG_INF_GRN("Sending DEVICE_UPDATED_ACK to device %s ID:%d for device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, device_type_str(pkt->dst_device_type), pkt->dst_device_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------**** Handlers Functions ****--------------------------------------------------------------------------- */

/* Handle joined network packet */
void handle_joined_network(const joined_network_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received JOINED_NETWORK from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, device_type_str(pkt->device_type), pkt->device_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate sender type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in JOINED_NETWORK from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    // Process only packets with status_success
    if (pkt->hdr.status != STATUS_SUCCESS) {
        return;
    }

    uint8_t status;

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Check if device is already in the network with this anchor/sensor
                status = check_mesh_storage(pkt->device_id);
                if (status == STATUS_SUCCESS) {
                    // Add to mesh storage
                    mesh_entry_t entry;
                    entry.device_type = pkt->device_type;
                    entry.device_id = pkt->device_id;
                    entry.serial_num = pkt->serial_num;
                    entry.version = pkt->version;
                    entry.connected_device_id = pkt->connected_device_id;
                    entry.hop_num = pkt->hop_num;
                    entry.sensor_count = pkt->sensor_count;
                    int err = storage_mesh_add(&entry);
                    if (err) {
                        LOG_ERR("Failed to store joined mesh device, err %d", err);
                    }
                    LOG_INF("%s ID:%d successfully joined network", device_type_str(pkt->device_type), pkt->device_id);
                } else if (status == STATUS_ALREADY_EXISTS) {
                    LOG_INF("%s ID:%d already in network, received JOINED_NETWORK with success", device_type_str(pkt->device_type), pkt->device_id);
                }
                // Send Ping Device
                ping_known_devices(pkt->device_id, status);
            } else {
                // Reject joined network except from anchor and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Upstream the Packet to Gateway if Anchor receives the JOINED_NETWORK packet from Sensor/Anchor
                joined_network_ack_t ack_pkt = {
                    .dst_device_id = pkt->device_id,
                    .dst_device_type = pkt->device_type,
                };
                send_joined_network_ack(&ack_pkt, dst_id, pkt->hdr.device_type, pkt->hdr.tracking_id, STATUS_SUCCESS);
                LOG_INF("Forwarding JOINED_NETWORK from %s ID:%d  for %s ID:%d upstream", device_type_str(pkt->hdr.device_type), dst_id, device_type_str(pkt->device_type), pkt->device_id);
                infra_entry_t entry;
                storage_infra_get(0, &entry);
                send_joined_network(pkt, entry.device_id, entry.device_type, STATUS_SUCCESS);
                return;
            } else {
                // Reject joined network except from anchor and sensor
                return;
            } 
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            // Sensor can't accept joined network only sends joined network, so reject any incoming joined network
            return;
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any joined network if this device has invalid type
            return;
        }
        break;
    }

    joined_network_ack_t ack_pkt = {
        .dst_device_id = pkt->device_id,
        .dst_device_type = pkt->device_type,
    };
    send_joined_network_ack(&ack_pkt, dst_id, pkt->hdr.device_type, pkt->hdr.tracking_id, status);

    return;
}

/* Handle joined network acknowledgment packet */
void handle_joined_network_ack(const joined_network_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{

    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received JOINED_NETWORK_ACK from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, device_type_str(pkt->dst_device_type), pkt->dst_device_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in JOINED_NETWORK_ACK from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    /* Remove the joined network tracker. */
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway can't accept joined network ack only sends joined network ack, so reject any incoming joined network ack
            return;
        }
        break;
        
        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Implement Later
            }
            else {
                // Reject joined network ack except from anchor and gateway
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                if (pkt->hdr.status != STATUS_SUCCESS) {
                    // Resend joined network packet. 
                    // Implement later
                    return;
                }
            } else {
                // Reject joined network ack except from anchor and gateway
                return;
            }
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any joined network ack if this device has invalid type
            return;
        }
        break;
    }

    return;
}

/* Handle ping device packet */
void handle_ping_device(const ping_device_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received PING_DEVICE from %s ID:%d with RSSI:%d (status: 0x%02x) (gen_id: %d)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status, pkt->dst_device_id);

    // Validate sender type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in PING_DEVICE from %d, ignoring", pkt->hdr.device_type, dst_id);
        return;
    }

    if (pkt->dst_device_id != 0 || pkt->dst_device_id != 0xFFFF) {
        for (int i = 0; i < infra_count; i++) {
            if (infra_devices[i].entry.device_id == pkt->dst_device_id && pkt->hdr.status != STATUS_ALREADY_EXISTS && pkt->hdr.status != STATUS_SUCCESS) {
                // Remove from Infra Storage
                send_ping_device(infra_devices[i].entry.device_id, infra_devices[i].entry.device_type, pkt->dst_device_id, pkt->hdr.status);
                storage_infra_remove(i);
                set_temp_id(infra_devices[i].entry.device_id);
                break;
            }
        }
        for (int i = 0; i < sensor_count; i++) {
            if (sensor_devices[i].entry.device_id == pkt->dst_device_id && pkt->hdr.status != STATUS_ALREADY_EXISTS && pkt->hdr.status != STATUS_SUCCESS) {
                // Remove from Sensor Storage
                send_ping_device(sensor_devices[i].entry.device_id, DEVICE_TYPE_SENSOR, pkt->dst_device_id, pkt->hdr.status);
                storage_sensor_remove(i);
                set_temp_id(sensor_devices[i].entry.device_id);
                break;
            }
        }
    }

    if (pkt->dst_device_id == get_device_id()) {
        if (pkt->hdr.status == STATUS_SUCCESS || pkt->hdr.status == STATUS_ALREADY_EXISTS) {
            mesh_pairing_set_pending_request();
        } else {
            // factory reset
            send_ping_ack(dst_id, pkt->hdr.device_type, pkt->dst_device_id, pkt->hdr.tracking_id, STATUS_SUCCESS);
            factory_reset();
            return;
        }
        
    }

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway never receives ping device because it's the root, but in case it receives ping device just ignore
            return;
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->dst_device_id != 0 && pkt->dst_device_id != 0xFFFF) {
                for (int i = 0; i < sensor_count; i++) {
                    if (sensor_devices[i].entry.device_id == pkt->dst_device_id) {
                        // Send device updated
                        LOG_INF("Sensor Count: %d", storage_sensor_count());
                        device_updated_t du_pkt = {
                            .device_type = get_device_type(),
                            .device_id = get_device_id(),
                            .serial_num = get_serial_number(),
                            .version = FIRMWARE_VERSION,
                            .connected_device_id = 0xFFFF,
                            .hop_num = get_hop_number(),
                            .sensor_count = storage_sensor_count(),
                        };
                        send_device_updated(&du_pkt, infra_devices[0].entry.device_id, infra_devices[0].entry.device_type, STATUS_SUCCESS);
                        break;
                    }
                }
            }
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY) {
                // Check if we need to update hop number
                if (update_infra_storage(dst_id, pkt->hop_num, rssi_2)) {
                    LOG_INF("Updated hop number for GATEWAY ID:%d to hop:%d based on PING_DEVICE", dst_id, get_hop_number());
                    device_updated_t du_pkt = {
                        .device_type = get_device_type(),
                        .device_id = get_device_id(),
                        .serial_num = get_serial_number(),
                        .version = FIRMWARE_VERSION,
                        .connected_device_id = 0xFFFF,
                        .hop_num = get_hop_number(),
                        .sensor_count = storage_sensor_count(),
                    };
                    send_device_updated(&du_pkt, dst_id, pkt->hdr.device_type, STATUS_SUCCESS);
                }
            } else if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if we need to update hop number
                if (update_infra_storage(dst_id, pkt->hop_num, rssi_2)) {
                    LOG_INF("Updated hop number for ANCHOR ID:%d to hop:%d based on PING_DEVICE", dst_id, get_hop_number());
                    device_updated_t du_pkt = {
                        .device_type = get_device_type(),
                        .device_id = get_device_id(),
                        .serial_num = get_serial_number(),
                        .version = FIRMWARE_VERSION,
                        .connected_device_id = 0xFFFF,
                        .hop_num = get_hop_number(),
                        .sensor_count = storage_sensor_count(),
                    };
                    send_device_updated(&du_pkt, dst_id, pkt->hdr.device_type, STATUS_SUCCESS);
                }
            } else {
                // Reject ping device except from gateway, anchor, and sensor
                return;
            }
            // Set the mesh time
            mesh_time_set(pkt->timestamp);
            set_mesh_devices_count(pkt->total_devices);
            ping_known_devices(pkt->dst_device_id, pkt->hdr.status);
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Update mesh time
                mesh_time_set(pkt->timestamp);
                update_infra_storage(dst_id, pkt->hop_num, rssi_2);
            } else {
                // Reject ping device except from gateway and anchor
                return;
            }
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any ping device if this device has invalid type
            return;
        }
        break;
    }
    // Send ACK back to the sender
    send_ping_ack(dst_id, pkt->hdr.device_type, pkt->dst_device_id, pkt->hdr.tracking_id, STATUS_SUCCESS);

    LOG_INF("Mesh Time %llu seconds", mesh_time_get() / 1000);
    return;
}

/* Handle ping ack packet */
void handle_ping_ack(const ping_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received PING_ACK from %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in PING_ACK from %d, ignoring", pkt->hdr.device_type, dst_id);
        return;
    }

    // Remove the ping tracker
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

    if (get_temp_id() != 0xFFFF && get_temp_id() == dst_id) {
        set_temp_id(0xFFFF);
        return;
    }

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Update Infra storage with new RSSI and hop number
                update_infra_storage(dst_id, pkt->hop_num, rssi_2);
            } else if (pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Update Sensor storage
                update_sensor_storage(dst_id, pkt->version);
            } else {
                // Reject ping ack except from anchor and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Check if we need to update hop number
                if (update_infra_storage(dst_id, pkt->hop_num, rssi_2)) {
                    LOG_INF("Updated hop number for ANCHOR ID:%d to hop:%d based on PING_ACK", dst_id, get_hop_number());
                    device_updated_t du_pkt = {
                        .device_type = get_device_type(),
                        .device_id = get_device_id(),
                        .serial_num = get_serial_number(),
                        .version = FIRMWARE_VERSION,
                        .connected_device_id = 0xFFFF,
                        .hop_num = get_hop_number(),
                        .sensor_count = storage_sensor_count(),
                    };
                    send_device_updated(&du_pkt, dst_id, pkt->hdr.device_type, STATUS_SUCCESS);
                }
            } else if (pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Update Sensor storage
                update_sensor_storage(dst_id, pkt->version);
            } else {
                // Reject ping ack except from gateway, anchor, and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            // Sensor will never receive ping ack, if it receives jut ignore it
            return;
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any ping ack if this device has invalid type
            return;
        }
        break;
    }

    LOG_INF("Mesh Time %llu seconds", mesh_time_get() / 1000);
    return;
}

/* Handle device updated packet */
void handle_device_updated(const device_updated_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received DEVICE_UPDATED from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, device_type_str(pkt->device_type), pkt->device_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate sender type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in DEVICE_UPDATED from %d, ignoring", pkt->hdr.device_type, dst_id);
        return;
    }

    // Process only packets with status_success
    if (pkt->hdr.status != STATUS_SUCCESS) {
        return;
    }

    uint8_t status;

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                status = check_mesh_storage(dst_id);
                if (status == STATUS_ALREADY_EXISTS) {
                    status = update_mesh_storage(dst_id, pkt->hop_num, pkt->version, pkt->connected_device_id, pkt->sensor_count);
                    LOG_INF("Updated mesh storage for device %s ID:%d based on DEVICE_UPDATED", device_type_str(pkt->hdr.device_type), pkt->hdr.device_id);
                } else {
                    LOG_WRN("Received DEVICE_UPDATED for device %s ID:%d which is not in mesh storage", device_type_str(pkt->hdr.device_type), pkt->hdr.device_id);
                    status = STATUS_NOT_FOUND;
                }
            } else{
                // Reject device updated except from anchor and sensor
                return;
            }
        }
        break;
        
        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Upstream the update to gateway
                status = STATUS_SUCCESS;
                LOG_INF("Forwarding DEVICE_UPDATED from %s ID:%d, updating infra storage", device_type_str(pkt->hdr.device_type), pkt->hdr.device_id);
                infra_entry_t entry;
                storage_infra_get(0, &entry);
                send_device_updated(pkt, entry.device_id, pkt->hdr.device_type, pkt->hdr.status);
            } else {
                // Reject device updated except from anchor and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            // Sensor will never receive device updated because only anchor and gateway can send device updated, so just ignore if received
            return;
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any device updated if this device has invalid type
            return;
        }
        break;
    }

    device_updated_ack_t ack_pkt = {
        .dst_device_id = pkt->hdr.device_id,
        .dst_device_type = pkt->hdr.device_type,
    };

    send_device_updated_ack(&ack_pkt, dst_id, pkt->hdr.device_type, pkt->hdr.tracking_id, status);

    return;
}

/* Handle device updated acknowledgment packet */
void handle_device_updated_ack(const device_updated_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received DEVICE_UPDATED_ACK from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, device_type_str(pkt->dst_device_type), pkt->dst_device_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in DEVICE_UPDATED_ACK from %d, ignoring", pkt->hdr.device_type, dst_id);
        return;
    }

    // Remove the ping tracker
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway will never receive device updated ack because only anchor can send device updated ack, so just ignore if received
            return;
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
            // Implement any necessary action if needed when anchor receives device updated ack from gateway/anchor
            }
            else {
                // Reject device updated ack except from anchor and gateway
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Implement any necessary action if needed when sensor receives device updated ack from gateway/anchor
            }
            else {
                // Reject device updated ack except from anchor and gateway
                return;
            }
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any device updated ack if this device has invalid type
            return;
        }
        break;
    }

    return;
}
