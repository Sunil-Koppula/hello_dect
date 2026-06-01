/* MESH SESSION HEADER */


#ifndef MESH_SESSION_H
#define MESH_SESSION_H

#include <stdint.h>
#include <stddef.h>
#include "../protocol.h"

/* TX helpers */
int send_joined_network(const joined_network_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t status);
int send_joined_network_ack(const joined_network_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status);
int send_ping_device(uint16_t dst_id, uint8_t dst_type, uint16_t gen_id, uint8_t status);
int send_ping_ack(uint16_t dst_id, uint8_t dst_type, uint16_t gen_id, uint8_t tracking_id, uint8_t status);
int send_device_updated(const device_updated_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t status);
int send_device_updated_ack(const device_updated_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status);

/* RX handlers — wire into each device's RX dispatch switch. */
void handle_joined_network(const joined_network_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_joined_network_ack(const joined_network_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_ping_device(const ping_device_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_ping_ack(const ping_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_device_updated(const device_updated_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_device_updated_ack(const device_updated_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);

#endif /* MESH_SESSION_H */