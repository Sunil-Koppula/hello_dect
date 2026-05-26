#ifndef PRODUCT_INFO_H
#define PRODUCT_INFO_H

#include <stdint.h>
#include "protocol.h"

#define PRODUCT_NAME "DECT NR+ PHY MESH"
#define FIRMWARE_VERSION 100

#define PING_TIMEOUT_MS      2 * 60 * 1000 /* 2 minutes timeout for known devices */
#define MAX_COMM_FAILURES    3

/* Runtime device type (set by product_info_init from GPIO pins). */
extern device_type_t DEVICE_TYPE;

/* 64-bit serial number: 0x00{device_id_16}00DEADBEEF */
extern uint64_t SERIAL_NUMBER;

/* Device hop number from gateway.
 * Gateway = 0, Anchor = min(infra hop) + 1, Sensor = 0xFF (no hop). */
extern uint8_t DEVICE_HOP_NUMBER;

/* Connected (upstream parent) device ID.
 * Gateway = 0, Anchor = best infra device ID, Sensor = paired device ID. */
extern uint16_t CONNECTED_DEVICE_ID;

/* Number of devices in the mesh network. */
extern uint16_t MESH_DEVICES_COUNT;

/* Read GPIO pins P0.21/P0.23 to set device type.
 * Must be called before any mesh operations. */
int product_info_init(void);

/* Update hop number (called after pairing for anchors). */
void device_info_update(void);

/* Update the list of known devices in the radio module. */
void update_known_devices(void);

/* Check if a device is known (connected or paired). */
bool is_known_device(uint16_t device_id);

/* Tick function to update known devices' last communication time. */
void known_devices_tick(void);

/* Update the last communication time for a known device. */
void known_device_update_comm_time(uint16_t device_id, bool is_successful_comm);

/* Ping all known devices at initialization. */
void ping_known_devices(uint16_t gen_id, uint8_t status);

/* Factory Reset */
void factory_reset(void);

#endif /* PRODUCT_INFO_H */
