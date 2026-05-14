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
int send_pair_request(void);
int send_pair_response(uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status);
int send_pair_confirm(uint16_t dst_id, uint8_t dst_type, uint8_t status);
int send_pair_ack(uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status);
int send_joined_network(const joined_network_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t status);
int send_joined_network_ack(const joined_network_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status);
int send_ping_device(uint16_t dst_id, uint8_t dst_type, uint8_t status);
int send_ping_ack(uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status);
int send_device_updated(const device_updated_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t status);
int send_device_updated_ack(const device_updated_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status);
int send_repair_request(uint16_t dst_id, uint8_t dst_type);
int send_repair_response(uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status);
int send_route_info(const route_info_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t status);
int send_route_info_ack(const route_info_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status);

/* RX handlers — wire into each device's RX dispatch switch. */
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
void handle_route_info(const route_info_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_route_info_ack(const route_info_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);

/* Compute hash. */
uint32_t compute_pair_hash(uint16_t dev_id, uint32_t random_num);

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

#endif /* MESH_H */