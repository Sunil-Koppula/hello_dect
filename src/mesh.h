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
int send_pair_response(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status, uint32_t hash, uint8_t hop_num);
int send_pair_confirm(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status);
int send_pair_ack(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status, uint8_t hop_num);
int send_joined_network(uint32_t handle,const joined_network_t *pkt, uint16_t dst_id, uint8_t tracking_id, uint8_t status);
int send_joined_network_ack(uint32_t handle, uint16_t dst_device_id, uint16_t dst_id, uint8_t tracking_id, uint8_t status);

void handle_pair_request(const pair_request_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_pair_response(const pair_response_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_pair_confirm(const pair_confirm_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_pair_ack(const pair_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_joined_network(const joined_network_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_joined_network_ack(const joined_network_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);

/* Call each main loop cycle to check response collection timer. */
void mesh_tick(void);

/* Returns true if response collection is in progress. */
bool mesh_is_collecting(void);

#endif /* MESH_H */