#ifndef PRODUCT_INFO_H
#define PRODUCT_INFO_H

#include <stdint.h>
#include "protocol.h"

#define PRODUCT_NAME "DECT NR+ PHY MESH"
#define FIRMWARE_VERSION 100

#define MESH_MAX_HOP        8
#define MAX_ANCHORS         8
#define MAX_SENSORS         64
#define MAX_DEVICES         512

/* Runtime device type (set by product_info_init from GPIO pins). */
extern device_type_t PRODUCT_DEVICE_TYPE;

/* 64-bit serial number: 0x00{device_id_16}00DEADBEEF */
extern uint64_t PRODUCT_SERIAL_NUMBER;

/* Device hop number from gateway.
 * Gateway = 0, Anchor = min(infra hop) + 1, Sensor = 0xFF (no hop). */
extern uint8_t PRODUCT_HOP_NUMBER;

/* Connected (upstream parent) device ID.
 * Gateway = 0, Anchor = best infra device ID, Sensor = paired device ID. */
extern uint16_t PRODUCT_CONNECTED_DEVICE_ID;

/* Read GPIO pins P0.21/P0.23 to set device type.
 * Must be called before any mesh operations. */
int product_info_init(void);

/* Update hop number (called after pairing for anchors). */
void product_info_update_hop(void);

#endif /* PRODUCT_INFO_H */
