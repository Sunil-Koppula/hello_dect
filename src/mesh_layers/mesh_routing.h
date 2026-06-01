/* MESH Routing */

#ifndef MESH_ROUTING_H
#define MESH_ROUTING_H

#include <stdint.h>
#include <stddef.h>
#include "../protocol.h"

/* TX helpers */
int send_route_discovery(const route_discovery_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t status);
int send_route_discovery_ack(const route_discovery_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status);
int send_route_info(const route_info_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t status);
int send_route_info_ack(const route_info_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t tracking_id, uint8_t status);

/* RX handlers — wire into each device's RX dispatch switch. */
void handle_route_discovery(const route_discovery_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_route_discovery_ack(const route_discovery_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_route_info(const route_info_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_route_info_ack(const route_info_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);

#endif /* MESH_ROUTING_H */