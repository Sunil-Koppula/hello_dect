#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include <stddef.h>
#include <zephyr/kernel.h>

/*
 * EEPROM partition layout (M24M02 — 256KB):
 *
 * Partition 1: Connected infrastructure (Gateway/Anchors)
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

#define STORAGE_HEADER_SIZE   4
#define STORAGE_MAGIC_0       0xDE
#define STORAGE_MAGIC_1       0xC7

/* Max entries per partition. */
#define STORAGE_PART1_MAX_ENTRIES ((STORAGE_PART1_SIZE - STORAGE_HEADER_SIZE) / sizeof(infra_entry_t))
#define STORAGE_PART2_MAX_ENTRIES ((STORAGE_PART2_SIZE - STORAGE_HEADER_SIZE) / sizeof(sensor_entry_t))
#define STORAGE_PART3_MAX_ENTRIES ((STORAGE_PART3_SIZE - STORAGE_HEADER_SIZE) / sizeof(mesh_entry_t))

/*
 * Partition 1 entry: Connected infrastructure devices (Gateway/Anchors).
 * Stored by: Gateway, Anchor.
 */
typedef struct {
	uint8_t  device_type;      /* device_type_t */
	uint16_t device_id;        /* short device ID */
	uint8_t  hop_num;          /* hop count from gateway */
} __attribute__((packed)) infra_entry_t;

/*
 * Partition 2 entry: Connected sensors.
 * Stored by: Gateway, Anchor.
 */
typedef struct {
	uint16_t device_id;        /* sensor short device ID */
} __attribute__((packed)) sensor_entry_t;

/*
 * Partition 3 entry: Full mesh network table.
 * Stored by: Gateway only.
 */
typedef struct {
	uint8_t  device_type;      /* device_type_t */
	uint16_t device_id;        /* short device ID */
	uint64_t serial_num;       /* 64-bit serial number */
	uint16_t connect_device_id;/* parent/connected device ID */
	uint8_t  hop_num;          /* hop count from gateway */
	uint8_t  sensor_count;     /* number of sensors connected to this device */
} __attribute__((packed)) mesh_entry_t;

/* Initialize storage (must be called once at boot). */
int storage_init(void);

/* Partition 1: Infrastructure devices (Gateway + Anchor). */
int storage_infra_add(const infra_entry_t *entry);
int storage_infra_get(uint16_t index, infra_entry_t *entry);
int storage_infra_count(void);
int storage_infra_clear(void);

/* Partition 2: Connected sensors (Gateway + Anchor). */
int storage_sensor_add(const sensor_entry_t *entry);
int storage_sensor_get(uint16_t index, sensor_entry_t *entry);
int storage_sensor_count(void);
int storage_sensor_clear(void);

/* Partition 3: Mesh network table (Gateway only). */
int storage_mesh_add(const mesh_entry_t *entry);
int storage_mesh_get(uint16_t index, mesh_entry_t *entry);
int storage_mesh_count(void);
int storage_mesh_clear(void);

#endif /* STORAGE_H */
