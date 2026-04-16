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
int send_pair_response(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status, uint32_t hash);
int send_pair_confirm(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status);
int send_pair_ack(uint32_t handle, uint16_t dst_id, uint8_t tracking_id, uint8_t status);

void handle_pair_request(const pair_request_t *pkt, uint16_t dst_id, int16_t rssi_2);

#endif /* MESH_H */