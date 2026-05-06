#ifndef PRODUCT_INFO_H
#define PRODUCT_INFO_H

#include <stdint.h>
#include "protocol.h"

#define PRODUCT_NAME "DECT NR+ PHY MESH"
#define FIRMWARE_VERSION 100

#define MAX_DEPTH           8
#define MAX_ANCHORS         8
#define MAX_SENSORS         64
#define MAX_DEVICES         512
#define MAX_KNOWN_DEVICES   (MAX_ANCHORS + MAX_SENSORS)

#define PING_TIMEOUT_MS      10 * 60 * 1000 /* 10 minutes timeout for known devices */
#define MAX_COMM_FAILURES    3

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

/* Connected Devices Information */
typedef struct {
    uint8_t device_type;
    uint16_t device_id;
    uint64_t last_comm_ms;
    uint8_t comm_failures;
    bool is_ping_packet_sent;
} known_device_t;

extern known_device_t known_devices[MAX_KNOWN_DEVICES];
extern uint8_t known_device_count;

/* Known Routes Information */
typedef struct {
    uint16_t device_id;
    uint16_t next_device_id[MAX_ANCHORS];
} known_route_t;

extern known_route_t known_route_table[MAX_DEVICES];
extern uint16_t known_route_count;

/* Read GPIO pins P0.21/P0.23 to set device type.
 * Must be called before any mesh operations. */
int product_info_init(void);

/* Update hop number (called after pairing for anchors). */
void product_info_update_hop(void);

/* Update the list of known devices in the radio module. */
void update_known_devices(void);

/* Check if a device is known (connected or paired). */
bool is_known_device(uint16_t device_id);

/* Get index of a known device by ID, or -1 if not found. */
int known_device_idx(uint16_t device_id);

/* Tick function to update known devices' last communication time. */
void known_devices_tick(void);

/* Update the last communication time for a known device. */
void known_device_update_comm_time(uint16_t device_id, bool is_successful_comm);

/* Ping all known devices at initialization. */
void ping_known_devices(void);

/* Get the next hop device ID for a given device ID. */
uint16_t get_next_hop_device_id(uint16_t device_id);

#endif /* PRODUCT_INFO_H */
