/*
 * Mesh helpers for DECT NR+ mesh network
 */

#ifndef MESH_H
#define MESH_H

#include <stdint.h>
#include <stddef.h>
#include "product_info.h"
#include "protocol.h"

/* TX helpers */
int send_pair_request(uint32_t handle, uint8_t tracking_id);
int send_pair_response(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status, uint8_t hop_num);
int send_pair_confirm(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status);
int send_pair_ack(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status, uint8_t hop_num);
int send_joined_network(uint32_t handle, const joined_network_t *pkt, uint16_t dst_id, uint8_t tracking_id, uint8_t status);
int send_joined_network_ack(uint32_t handle, uint16_t dst_device_id, uint16_t dst_id, uint8_t tracking_id, uint8_t status);
int send_ping_device(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status);
int send_ping_ack(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status);
int send_device_updated(uint32_t handle, const device_updated_t *pkt, uint16_t dst_id, uint8_t tracking_id, uint8_t status);
int send_device_updated_ack(uint32_t handle, uint16_t dst_device_id, uint16_t dst_id, uint8_t tracking_id, uint8_t status);
int send_repair_request(uint32_t handle, uint16_t dst_id, uint8_t tracking_id);
int send_repair_response(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status);
int send_sync_time(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status);
int send_sync_time_ack(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status);

void handle_pair_request(const pair_request_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_pair_response(const pair_response_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_pair_confirm(const pair_confirm_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_pair_ack(const pair_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_joined_network(const joined_network_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_joined_network_ack(const joined_network_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_ping_device(const ping_device_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_ping_ack(const ping_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_device_updated(const device_updated_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_device_updated_ack(const device_updated_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_repair_request(const repair_request_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_repair_response(const repair_response_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_sync_time(const sync_time_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_sync_time_ack(const sync_time_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);

/* Call each main loop cycle to check response collection timer. */
void mesh_tick(void);

/* Returns true if response collection is in progress. */
bool mesh_is_collecting(void);

/* Mesh time — synchronized milliseconds across the network.
 * Gateway sets the baseline at startup; anchors/sensors track the gateway's
 * value via SYNC_TIME broadcasts. Always read via mesh_time_get(); it
 * accounts for local elapsed time since the last sync.
 */
void mesh_time_init(void);
uint64_t mesh_time_get(void);
void mesh_time_set(uint64_t remote_mesh_time);

/* Call each main loop cycle. Logs the current mesh_time once per 5-minute
 * boundary crossing (5, 10, 15... minutes). No-op between boundaries.
 */
void mesh_time_check_milestone(void);

#endif /* MESH_H */