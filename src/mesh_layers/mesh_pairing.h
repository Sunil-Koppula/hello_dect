/* MESH Pairing Header */

#ifndef MESH_PAIRING_H
#define MESH_PAIRING_H

#include <stdint.h>
#include <stddef.h>
#include "../protocol.h"

/* Call each main loop cycle to check response collection timer. */
void mesh_pairing_tick(void);

void mesh_pairing_set_pending_request(void);

/* TX helpers */
int send_pair_request(bool include_tracker);
int send_pair_response(uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status);
int send_pair_confirm(uint16_t dst_id, uint8_t dst_type, uint8_t status);
int send_pair_ack(uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status);
int send_repair_request(uint16_t dst_id, uint8_t dst_type);
int send_repair_response(uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status);

/* RX handlers — wire into each device's RX dispatch switch. */
void handle_pair_request(const pair_request_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_pair_response(const pair_response_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_pair_confirm(const pair_confirm_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_pair_ack(const pair_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_repair_request(const repair_request_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_repair_response(const repair_response_t *pkt, uint16_t dst_id, int16_t rssi_2);

#endif /* MESH_PAIRING_H */