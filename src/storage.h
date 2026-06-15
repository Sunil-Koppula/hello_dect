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
 *   Offset 0x00000, 128B — stored by Gateway + Anchor
 *
 * Partition 2: Connected sensors
 *   Offset 0x00080, 1KB — stored by Gateway + Anchor
 *
 * Partition 3: Mesh network table (full topology)
 *   Offset 0x00480, 12KB — stored by Gateway only
 *
 * Each partition starts with a 4-byte header:
 *   [0-1] magic (0xDE, 0xC7)
 *   [2-3] entry count (uint16_t LE)
 */

/* Partition base addresses. */

#define STORAGE_INFRA_OFFSET			0x00000
#define STORAGE_INFRA_SIZE				0x00080  /* 128B */

#define STORAGE_SENSOR_OFFSET			0x00080
#define STORAGE_SENSOR_SIZE				0x00400  /* 1KB */

#define STORAGE_MESH_OFFSET				0x00480
#define STORAGE_MESH_SIZE				0x03000  /* 12KB */

#define STORAGE_HEADER_SIZE		4
#define STORAGE_MAGIC_0			0xDE
#define STORAGE_MAGIC_1			0xC7

/*
 * Partition 1 entry: Connected infrastructure devices (Gateway/Anchors).
 * Stored by: Gateway, Anchor.
 */
typedef struct {
	uint8_t  device_type;      /* device_type_t */
	uint16_t device_id;        /* short device ID */
	uint8_t  hop_num;          /* hop count from gateway */
	uint16_t version;          /* firmware version */
	int16_t rssi_2;            /* RSSI in 0.5 dBm units (signed; typically negative) */
	uint32_t crc32;            /* CRC32 over all preceding bytes; must be last */
} __attribute__((packed)) infra_entry_t;

// Stored in Internal RAM for fast access
typedef struct {
	infra_entry_t entry;		/* the actual infra entry data */
	uint64_t last_comm_ms;     	/* timestamp of last communication with this device */
	uint8_t comm_failures;     	/* consecutive communication failures */
	bool is_ping_packet_sent; 	/* whether a ping packet has been sent to this device after the last communication */
} infra_t;

extern infra_t infra_devices[MAX_ANCHORS];
extern uint8_t infra_count;

/*
 * Partition 2 entry: Connected sensors.
 * Stored by: Gateway, Anchor.
 */
typedef struct {
	uint16_t device_id;        /* sensor short device ID */
	uint16_t version;          /* sensor firmware version */
	uint32_t crc32;            /* CRC32 over all preceding bytes; must be last */
} __attribute__((packed)) sensor_entry_t;

// Stored in Internal RAM for fast access
typedef struct {
	sensor_entry_t entry;		/* the actual sensor entry data */
	uint64_t last_comm_ms;     /* timestamp of last communication with this sensor */
	uint8_t comm_failures;     /* consecutive communication failures */
	bool is_ping_packet_sent; /* whether a ping packet has been sent to this sensor after the last communication */
} sensor_t;

extern sensor_t sensor_devices[MAX_SENSORS];
extern uint8_t sensor_count;

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
	uint32_t crc32;					/* CRC32 over all preceding bytes; must be last */
} __attribute__((packed)) mesh_entry_t;

typedef struct {
	uint8_t  device_type;      		/* device_type_t */
	uint16_t device_id;        		/* short device ID */
	uint64_t serial_num;       		/* 64-bit serial number */
	uint16_t connected_device_id;	/* parent/connected device ID */
	uint8_t  hop_num;          		/* hop count from gateway */
} mesh_t;

extern mesh_t mesh_devices[MAX_DEVICES];
extern uint16_t mesh_count;

/* Compile-time partition-capacity checks. If any MAX_* constant grows beyond
 * what the partition can hold, the build fails here instead of at runtime. */
_Static_assert(STORAGE_HEADER_SIZE + (uint32_t)sizeof(infra_entry_t)  * MAX_ANCHORS  <= STORAGE_INFRA_SIZE,
               "INFRA partition too small for MAX_ANCHORS infra_entry_t slots");
_Static_assert(STORAGE_HEADER_SIZE + (uint32_t)sizeof(sensor_entry_t) * MAX_SENSORS  <= STORAGE_SENSOR_SIZE,
               "SENSOR partition too small for MAX_SENSORS sensor_entry_t slots");
_Static_assert(STORAGE_HEADER_SIZE + (uint32_t)sizeof(mesh_entry_t)   * MAX_DEVICES  <= STORAGE_MESH_SIZE,
               "MESH partition too small for MAX_DEVICES mesh_entry_t slots");

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

#endif /* STORAGE_H */
