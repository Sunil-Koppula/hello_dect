#ifndef PRODUCT_INFO_H
#define PRODUCT_INFO_H

#include <stdint.h>
#include "protocol.h"

#define PRODUCT_NAME "DECT NR+ PHY MESH"
#define FIRMWARE_VERSION 100

typedef struct {
    device_type_t device_type;
    uint16_t device_id;
    uint64_t serial_number;
    uint16_t connected_device_id;
    uint8_t hop_num;
    uint16_t firmware_version;
    uint16_t mesh_devices_count;
} device_info_t;

/* Read GPIO pins P0.21/P0.23 to set device type.
 * Must be called before any mesh operations. */
int product_info_init(void);  // Change it device_init

/**
 * @brief Sets device type based on GPIO pin readings.
 * 
 * @param type The device type to set (gateway, anchor, sensor).
 * @return None
 */
void set_device_type(device_type_t type);

/**
 * @brief Gets the current device type.
 * 
 * @return The current device type (gateway, anchor, sensor).
 */
device_type_t get_device_type(void);

/**
 * @brief Sets the device ID.
 * 
 * @param device_id The device ID to set.
 * @return None
 */
void set_device_id(uint16_t device_id);

/**
 * @brief Gets the current device ID.
 * 
 * @return The current device ID.
 */
uint16_t get_device_id(void);

/**
 * @brief Sets the serial number.
 * 
 * @param serial_num The serial number to set.
 * @return None
 */
void set_serial_number(uint64_t serial_num);

/**
 * @brief Gets the current serial number.
 * 
 * @return The current serial number.
 */
uint64_t get_serial_number(void);

/**
 * @brief Sets the connected device ID (upstream parent).
 * 
 * @param connected_device_id The connected device ID to set.
 * @return None
 */
void set_connected_device_id(uint16_t connected_device_id);

/**
 * @brief Gets the connected device ID (upstream parent).
 * 
 * @return The connected device ID.
 */
uint16_t get_connected_device_id(void);

/**
 * @brief Sets the hop number from gateway.
 * 
 * @param hop_num The hop number to set.
 * @return None
 */
void set_hop_number(uint8_t hop_num);

/**
 * @brief Gets the hop number from gateway.
 * 
 * @return The hop number from gateway.
 */
uint8_t get_hop_number(void);

/**
 * @brief Sets the firmware version.
 * 
 * @param firmware_version The firmware version to set.
 * @return None
 */
void set_firmware_version(uint16_t firmware_version);

/**
 * @brief Gets the firmware version.
 * 
 * @return The firmware version.
 */
uint16_t get_firmware_version(void);

/**
 * @brief Sets the mesh devices count.
 * 
 * @param mesh_devices_count The mesh devices count to set.
 * @return None
 */
void set_mesh_devices_count(uint16_t mesh_devices_count);

/**
 * @brief Gets the mesh devices count.
 * 
 * @return The mesh devices count.
 */
uint16_t get_mesh_devices_count(void);

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

/* Get the hop number for a device. */
uint8_t get_hop_num(uint16_t device_id, uint8_t device_type);

/* Factory Reset */
void factory_reset(void);

#endif /* PRODUCT_INFO_H */
