

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
#include "../config.h"
#include "../data.h"

LOG_MODULE_REGISTER(mesh_routing, CONFIG_MESH_ROUTING_LOG_LEVEL);

static route_info_t build_route_info(const route_info_t *pkt)
{
    route_info_t info_pkt = {
        .device_id = pkt->device_id,
        .device_type = pkt->device_type,
        .data_type = pkt->data_type,
        .data_id = pkt->data_id,
    };

    for (int i = 0; i < MAX_DEPTH; i++) {
        if (info_pkt.route_info[i].device_id == 0xFFFF && info_pkt.route_info[i].hop_num == 0xFF) {
            info_pkt.route_info[i].device_id = get_device_id();
            info_pkt.route_info[i].hop_num = get_hop_number();
            break;
        }
    }
    return info_pkt;
}

static config_t build_config_pkt(const route_info_t *pkt)
{
    LOG_INF("Before sorting");
    for (int i = 0; i < MAX_DEPTH; i++) {
        if (pkt->route_info[i].device_id != 0xFFFF) {
            LOG_INF("   %d: Device ID:%d, Hop Num:%d", i, pkt->route_info[i].device_id, pkt->route_info[i].hop_num);
        }
    }

    int idx = pkt->data_id;
    uint32_t addr = PSRAM_CONFIG_BASE + ((uint32_t)idx * MAX_CONFIG_SIZE);

    config_t config_pkt = {
        .dst_device_id = pkt->device_id,
        .dst_device_type = pkt->device_type,
        .data_type = pkt->data_type,
        .data_id = config_slots[idx].config_id,
        .config_len = config_slots[idx].config_len,
        .config_crc32 = config_slots[idx].config_crc32,
    };

    int err = psram_read(addr, config_pkt.config, config_slots[idx].config_len);
    if (err) {
        LOG_ERR("psram_read @0x%06x failed (%d)", addr, err);
        config_pkt.config_len = 0;
        return config_pkt;
    }

    for (idx = 0; idx < MAX_DEPTH; idx++) {
        if (pkt->route_info[idx].device_id == 0xFFFF) {
            break;
        }
    }
    for (int i = 0; i <= idx - 1; i++) {
        config_pkt.route_info[i] = pkt->route_info[idx - 1 - i];
    }
    for (int i = idx; i < MAX_DEPTH; i++) {
        config_pkt.route_info[i].device_id = 0xFFFF;
        config_pkt.route_info[i].hop_num = 0xFF;
    }
    LOG_INF("After sorting");
    for (int i = 0; i < MAX_DEPTH; i++) {
        if (config_pkt.route_info[i].device_id != 0xFFFF) {
            LOG_INF("   %d: Device ID:%d, Hop Num:%d", i, config_pkt.route_info[i].device_id, config_pkt.route_info[i].hop_num);
        }
    }

    // Print Config pkt except config data for debugging
    LOG_INF("Built config packet for device %s ID:%d with data type 0x%02x, data ID %d, config len %d, crc32 0x%08x",
        device_type_str(config_pkt.dst_device_type), config_pkt.dst_device_id, config_pkt.data_type, config_pkt.data_id, config_pkt.config_len, config_pkt.config_crc32);

    return config_pkt;
}

static report_received_t build_report_received_pkt(const route_info_t *pkt)
{
    LOG_INF("Before sorting");
    for (int i = 0; i < MAX_DEPTH; i++) {
        if (pkt->route_info[i].device_id != 0xFFFF) {
            LOG_INF("   %d: Device ID:%d, Hop Num:%d", i, pkt->route_info[i].device_id, pkt->route_info[i].hop_num);
        }
    }

    report_received_t recv_pkt = {
        .gen_device_id = pkt->device_id,
        .data_id = pkt->data_id,
        .hdr.status = STATUS_SUCCESS,
    };

    int idx = 0;
    for (idx = 0; idx < MAX_DEPTH; idx++) {
        if (pkt->route_info[idx].device_id == 0xFFFF) {
            break;
        }
    }
    for (int i = 0; i <= idx - 1; i++) {
        recv_pkt.route_info[i] = pkt->route_info[idx - 1 - i];
    }
    for (int i = idx; i < MAX_DEPTH; i++) {
        recv_pkt.route_info[i].device_id = 0xFFFF;
        recv_pkt.route_info[i].hop_num = 0xFF;
    }
    LOG_INF("After sorting");
    for (int i = 0; i < MAX_DEPTH; i++) {
        if (recv_pkt.route_info[i].device_id != 0xFFFF) {
            LOG_INF("   %d: Device ID:%d, Hop Num:%d", i, recv_pkt.route_info[i].device_id, recv_pkt.route_info[i].hop_num);
        }
    }

     // Print report received pkt for debugging
    LOG_INF("Built report received packet for device %s ID:%d with data type 0x%02x, data ID %d",
        device_type_str(DEVICE_TYPE_SENSOR), recv_pkt.gen_device_id, DATA_TYPE_REPORT, recv_pkt.data_id);

    return recv_pkt;
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------------**** TX Helpers ****------------------------------------------------------------------------------- */

/* Send route discovery packet. */
int send_route_discovery(const route_discovery_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t status)
{
    route_discovery_t packet = {
        .hdr = {
            .packet_type = PACKET_ROUTE_DISCOVERY,
            .device_type = get_device_type(),
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracker_next_id(),
            .device_id = dst_id,
            .status = status,
        },
        .device_id = pkt->device_id,
        .device_type = pkt->device_type,
        .hop_num = pkt->hop_num,
        .data_type = pkt->data_type,
        .data_id = pkt->data_id,
    };

    // Add tracker entry for retries
    tracker_add(dst_id, get_device_id(), packet.hdr.tracking_id, PACKET_ROUTE_DISCOVERY, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF_GRN("Sending ROUTE_DISCOVERY to device %s ID:%d for device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, device_type_str(pkt->device_type), pkt->device_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send route discovery acknowledgment packet. */
int send_route_discovery_ack(const route_discovery_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status)
{
    route_discovery_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_ROUTE_DISCOVERY_ACK,
            .device_type = get_device_type(),
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .device_id = pkt->device_id,
        .device_type = pkt->device_type,
        .hop_num = pkt->hop_num,
    };

    LOG_INF_GRN("Sending ROUTE_DISCOVERY_ACK to device %s ID:%d for device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, device_type_str(pkt->device_type), pkt->device_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send route info packet. */
int send_route_info(const route_info_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t status)
{
    route_info_t packet = {
        .hdr = {
            .packet_type = PACKET_ROUTE_INFO,
            .device_type = get_device_type(),
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracker_next_id(),
            .device_id = dst_id,
            .status = status,
        },
        .device_id = pkt->device_id,
        .device_type = pkt->device_type,
        .data_type = pkt->data_type,
        .data_id = pkt->data_id,
    };
    memcpy(packet.route_info, pkt->route_info, sizeof(packet.route_info));

    // Add tracker entry for retries
    tracker_add(dst_id, get_device_id(), packet.hdr.tracking_id, PACKET_ROUTE_INFO, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, &packet, sizeof(packet));

    LOG_INF_GRN("Sending ROUTE_INFO to device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* Send route info acknowledgment packet. */
int send_route_info_ack(const route_info_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status)
{
    route_info_ack_t packet = {
        .hdr = {
            .packet_type = PACKET_ROUTE_INFO_ACK,
            .device_type = get_device_type(),
            .priority = PACKET_PRIORITY_HIGH,
            .tracking_id = tracking_id,
            .device_id = dst_id,
            .status = status,
        },
        .device_id = pkt->device_id,
        .device_type = pkt->device_type,
        .data_type = pkt->data_type,
        .data_id = pkt->data_id,
    };

    LOG_INF_GRN("Sending ROUTE_INFO_ACK to device %s ID:%d (status: 0x%02x)", device_type_str(dst_type), dst_id, status);
    return tx_queue_put(&packet, sizeof(packet), packet.hdr.priority);
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------**** Handlers Functions ****--------------------------------------------------------------------------- */

/* Handle route discovery packet */
void handle_route_discovery(const route_discovery_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received ROUTE_DISCOVERY from %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate sender type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in ROUTE_DISCOVERY from %d, ignoring", pkt->hdr.device_type, dst_id);
        return;
    }

    // Process only packets with status_success
    if (pkt->hdr.status != STATUS_SUCCESS) {
        return;
    }

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway will never receive route discovery because it's the root, so just ignore if received
            return;
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_GATEWAY) {
                if (pkt->device_type == DEVICE_TYPE_ANCHOR && pkt->device_id == get_device_id()) {
                    // Implement Later: Found route
                    LOG_INF("Found route to ANCHOR ID:%d", pkt->device_id);
                    // Build route info packet and send to gateway
                    route_info_t info_pkt = {
                        .device_id = pkt->device_id,
                        .device_type = pkt->device_type,
                        .data_type = pkt->data_type,
                        .data_id = pkt->data_id,
                        .route_info[0].device_id = get_device_id(),
                        .route_info[0].hop_num = get_hop_number(),
                    };
                    for (int i = 1; i < MAX_DEPTH; i++) {
                        info_pkt.route_info[i].device_id = 0XFFFF;
                        info_pkt.route_info[i].hop_num = 0xFF;
                    }
                    send_route_info(&info_pkt, infra_devices[0].entry.device_id, infra_devices[0].entry.device_type, STATUS_SUCCESS);
                } else if (pkt->device_type == DEVICE_TYPE_SENSOR) {
                    // Check in sensor storage
                    for (int i = 0; i < sensor_count; i++) {
                        if (sensor_devices[i].entry.device_id == pkt->device_id) {
                            LOG_INF("Found route to SENSOR ID:%d", pkt->device_id);
                            // Build route info packet and send to gateway
                            route_info_t info_pkt = {
                                .device_id = pkt->device_id,
                                .device_type = pkt->device_type,
                                .data_type = pkt->data_type,
                                .data_id = pkt->data_id,
                                .route_info[0].device_id = get_device_id(),
                                .route_info[0].hop_num = get_hop_number(),
                            };
                            for (int j = 1; j < MAX_DEPTH; j++) {
                                info_pkt.route_info[j].device_id = 0XFFFF;
                                info_pkt.route_info[j].hop_num = 0xFF;
                            }
                            send_route_info(&info_pkt, infra_devices[0].entry.device_id, infra_devices[0].entry.device_type, STATUS_SUCCESS);
                            break;
                        }
                    }
                } else {
                    // Forward the route discovery packet
                    for (int i = 0; i < infra_count; i++) {
                        if (infra_devices[i].entry.hop_num <= pkt->hop_num && infra_devices[i].entry.hop_num > get_hop_number() && infra_devices[i].entry.device_id != dst_id) {
                            send_route_discovery(pkt, infra_devices[i].entry.device_id, infra_devices[i].entry.device_type, pkt->hdr.status);
                            LOG_INF("Forwarding ROUTE_DISCOVERY for device %s ID:%d to %s ID:%d", device_type_str(DEVICE_TYPE_ANCHOR), pkt->device_id, device_type_str(infra_devices[i].entry.device_type), infra_devices[i].entry.device_id);
                        }
                    }
                }                
            } else {
                // Reject route discovery except from anchor and gateway
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            // Sensor will never receive route discovery because only anchor can send route discovery to sensor, so just ignore if received
            return;
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any route discovery if this device has invalid type
            return;
        }
        break;
    }
    route_discovery_ack_t ack_pkt = {
        .device_id = pkt->device_id,
        .device_type = pkt->device_type,
        .hop_num = pkt->hop_num,
    };

    send_route_discovery_ack(&ack_pkt, dst_id, pkt->hdr.device_type, pkt->hdr.tracking_id, pkt->hdr.status);
    return;
}

/* Handle route discovery acknowledgment packet */
void handle_route_discovery_ack(const route_discovery_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received ROUTE_DISCOVERY_ACK from %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in ROUTE_DISCOVERY_ACK from %d, ignoring", pkt->hdr.device_type, dst_id);
        return;
    }

    /* Remove the tracker. */
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

    return;
}

/* Handle route info packet */
void handle_route_info(const route_info_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received ROUTE_INFO from %s ID:%d with RSSI:%d (Data type: %d, status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, (rssi_2 / 2), pkt->data_type, pkt->hdr.status);

    // Validate sender type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in ROUTE_INFO from %d, ignoring", pkt->hdr.device_type, dst_id);
        return;
    }

    // Process only packets with status_success
    if (pkt->hdr.status != STATUS_SUCCESS) {
        return;
    }

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Start sending the Data (Configs for that device)
                LOG_INF("Received route info for device %s ID:%d (Data type: %d)", device_type_str(pkt->device_type), pkt->device_id, pkt->data_type);
                if (pkt->data_type == DATA_TYPE_CONFIG) { 
                    // Build config packet and send
                    config_t config_pkt = build_config_pkt(pkt);
                    if (config_pkt.config_len == 0) {
                        LOG_WRN("No config data for device %s ID:%d, cannot send config", device_type_str(pkt->device_type), pkt->device_id);
                        return;
                    }
                    send_config(&config_pkt, dst_id, DEVICE_TYPE_ANCHOR, STATUS_SUCCESS);
                } else if (pkt->data_type == DATA_TYPE_REPORT) {
                    // Build report received packet and send
                    report_received_t recv_pkt = build_report_received_pkt(pkt);
                    send_report_received(&recv_pkt, dst_id, DEVICE_TYPE_ANCHOR);
                }
            } else {
                // Reject route info except from anchor
                return;
            }
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_id == DEVICE_TYPE_GATEWAY) {
                route_info_t info_pkt = build_route_info(pkt);
                send_route_info(&info_pkt, infra_devices[0].entry.device_id, infra_devices[0].entry.device_type, STATUS_SUCCESS);
            } else {
                // Reject route info except from anchor and gateway
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            // Sensor will never receive route info because only anchor can send route info to sensor, so just ignore if received
            return;
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any route info if this device has invalid type
            return;
        }
        break;
    }

    route_info_ack_t ack_pkt = {
        .device_id = pkt->device_id,
        .device_type = pkt->device_type,
        .data_type = pkt->data_type,
        .data_id = pkt->data_id,
    };

    send_route_info_ack(&ack_pkt, dst_id, pkt->hdr.device_type, pkt->hdr.tracking_id, pkt->hdr.status);
    return;
}

/* Handle route info acknowledgment packet */
void handle_route_info_ack(const route_info_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    LOG_INF_MAG("   Received ROUTE_INFO_ACK from %s ID:%d for device %s ID:%d with RSSI:%d (status: 0x%02x)", device_type_str(pkt->hdr.device_type), dst_id, device_type_str(pkt->device_type), pkt->device_id, (rssi_2 / 2), pkt->hdr.status);

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in ROUTE_INFO_ACK from %d, ignoring", pkt->hdr.device_type, dst_id);
        return;
    }

    /* Remove the tracker. */
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway will never receive route info ack because only anchor can send route info ack to gateway, so just ignore if received
            return;
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // Nothing to do for now,
                return;
            } else {
                // Reject route info ack except from anchor
                return;
            }
        }
        break;
        
        case DEVICE_TYPE_SENSOR:
        {
            // Sensor will never receive route info ack because only anchor can send route info ack to sensor, so just ignore if received
            return;
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any route info ack if this device has invalid type
            return;
        }
        break;
    }
    return;
}