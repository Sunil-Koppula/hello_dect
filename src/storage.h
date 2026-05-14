#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include <stddef.h>
#include <zephyr/kernel.h>
#include "product_info.h"

/*
 * EEPROM partition layout (M24M02 — 256KB):
 *
 * Partition 1: Connected infrastructure (Gateway/Anchors/Sensors)
 *   Offset 0x00000, 4KB — stored by Gateway + Anchor
 *
 * Partition 2: Connected sensors
 *   Offset 0x01000, 4KB — stored by Gateway + Anchor
 *
 * Partition 3: Mesh network table (full topology)
 *   Offset 0x02000, 16KB — stored by Gateway only
 *
 * Each partition starts with a 4-byte header:
 *   [0-1] magic (0xDE, 0xC7)
 *   [2-3] entry count (uint16_t LE)
 */

/* Partition base addresses. */
#define STORAGE_PART1_OFFSET  0x00000
#define STORAGE_PART1_SIZE    0x01000  /* 4KB */

#define STORAGE_PART2_OFFSET  0x01000
#define STORAGE_PART2_SIZE    0x01000  /* 4KB */

#define STORAGE_PART3_OFFSET  0x02000
#define STORAGE_PART3_SIZE    0x04000  /* 16KB */

#define STORAGE_PART4_OFFSET  0x06000
#define STORAGE_PART4_SIZE    0x08000  /* 32KB */

#define STORAGE_HEADER_SIZE   4
#define STORAGE_MAGIC_0       0xDE
#define STORAGE_MAGIC_1       0xC7

/*
 * Partition 1 entry: Connected infrastructure devices (Gateway/Anchors).
 * Stored by: Gateway, Anchor.
 */
typedef struct {
	uint8_t  device_type;      /* device_type_t */
	uint16_t device_id;        /* short device ID */
	uint8_t  hop_num;          /* hop count from gateway */
	uint16_t version;          /* firmware version */
	uint16_t rssi_2;           /* RSSI */
} __attribute__((packed)) infra_entry_t;

// Stored in Internal RAM for fast access
typedef struct {
	uint8_t  device_type;      /* device_type_t */
	uint16_t device_id;        /* short device ID */
	uint64_t last_comm_ms;     /* timestamp of last communication with this device */
	uint8_t comm_failures;     /* consecutive communication failures */
	bool is_ping_packet_sent; /* whether a ping packet has been sent to this device after the last communication */
} infra_known_entry_t;

extern infra_known_entry_t infra_known_devices[MAX_ANCHORS];
extern uint8_t infra_known_device_count;

/*
 * Partition 2 entry: Connected sensors.
 * Stored by: Gateway, Anchor.
 */
typedef struct {
	uint16_t device_id;        /* sensor short device ID */
	uint16_t version;          /* sensor firmware version */
} __attribute__((packed)) sensor_entry_t;

// Stored in Internal RAM for fast access
typedef struct {
	uint16_t device_id;        /* sensor short device ID */
	uint64_t last_comm_ms;     /* timestamp of last communication with this sensor */
	uint8_t comm_failures;     /* consecutive communication failures */
	bool is_ping_packet_sent; /* whether a ping packet has been sent to this sensor after the last communication */
} sensor_known_entry_t;

extern sensor_known_entry_t sensor_known_devices[MAX_SENSORS];
extern uint8_t sensor_known_device_count;

/*
 * Partition 3 entry: Full mesh network table.
 * Stored by: Gateway only.
 */
typedef struct {
	uint8_t  device_type;      		/* device_type_t */
	uint16_t device_id;        		/* short device ID */
	uint64_t serial_num;       		/* 64-bit serial number */
	uint16_t version;          		/* device firmware version */
	uint16_t connected_device_id;	/* parent/connected device ID */
	uint8_t  hop_num;          		/* hop count from gateway */
	uint8_t  sensor_count;     		/* number of sensors connected to this device */
} __attribute__((packed)) mesh_entry_t;

/* 
 * Partition 4: Routing table for mesh network.
 * Stored by: Gateway, Anchor.
 */
typedef struct {
	uint16_t next_hop_id;    		/* short device ID of the next hop towards this device */
	uint8_t route_length;     		/* number of hops to this device */
	uint16_t avg_rssi_2;			/* Average RSSI to this device (in units of 0.5 dBm) */
} __attribute__((packed)) route_t;

typedef struct {
	uint8_t device_type;      			/* device_type_t */
	uint16_t device_id;      			/* short device ID */
	uint8_t route_count;     			/* number of routes to this device (for multipath) */
	route_t route_info[MAX_ANCHORS];    /* routing info to reach this device */
} __attribute__((packed)) route_entry_t;

/* Known Routes Information */
typedef struct {
    uint16_t device_id;
    uint16_t next_device_id;
} known_route_t;

extern known_route_t known_route_table[MAX_DEVICES];
extern uint16_t known_route_count;

/* Initialize storage (must be called once at boot). */
int storage_init(void);

/* Partition 1: Infrastructure devices (Gateway + Anchor). */
int storage_infra_add(const infra_entry_t *entry);
int storage_infra_update(uint16_t index, const infra_entry_t *entry);
int storage_infra_get(uint16_t index, infra_entry_t *entry);
int storage_infra_remove(uint16_t index);
int storage_infra_count(void);
int storage_infra_clear(void);

/* Partition 2: Connected sensors (Gateway + Anchor). */
int storage_sensor_add(const sensor_entry_t *entry);
int storage_sensor_update(uint16_t index, const sensor_entry_t *entry);
int storage_sensor_get(uint16_t index, sensor_entry_t *entry);
int storage_sensor_remove(uint16_t index);
int storage_sensor_count(void);
int storage_sensor_clear(void);

/* Partition 3: Mesh network table (Gateway only). */
int storage_mesh_add(const mesh_entry_t *entry);
int storage_mesh_update(uint16_t index, const mesh_entry_t *entry);
int storage_mesh_get(uint16_t index, mesh_entry_t *entry);
int storage_mesh_remove(uint16_t index);
int storage_mesh_count(void);
int storage_mesh_clear(void);

/* Partition 4: Routing table for mesh network (Gateway + Anchor). */
int storage_route_add(uint16_t dst_id, uint16_t device_id, uint8_t device_type, uint8_t route_len, int16_t avg_rssi_2);
int storage_route_get(uint16_t index, route_entry_t *entry);
int storage_route_remove(uint16_t device_id, uint16_t dst_id);
int storage_route_count(void);
int storage_route_clear(void);

#endif /* STORAGE_H */
