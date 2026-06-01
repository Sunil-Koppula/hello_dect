/*
 * Product info — reads GPIO straps to determine device type at runtime.
 *
 * Pin mapping (active-low, internal pull-up):
 *   P0.21  P0.23  →  type
 *     0      0       Gateway
 *     0      1       Anchor
 *     1      0       Sensor
 *     1      1       Sensor (reserved, defaults to Sensor)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include "main_sub.h"
#include "product_info.h"
#include "tracker.h"
#include "radio.h"
#include "mesh.h"
#include "storage.h"
#include "mesh_layers/mesh_pairing.h"
#include "mesh_layers/mesh_session.h"

LOG_MODULE_REGISTER(product_info, CONFIG_PRODUCT_INFO_LOG_LEVEL);

#define DEVTYPE_PIN0_NODE DT_NODELABEL(devtype_pin0)
#define DEVTYPE_PIN1_NODE DT_NODELABEL(devtype_pin1)

static const struct gpio_dt_spec pin0 = GPIO_DT_SPEC_GET(DEVTYPE_PIN0_NODE, gpios);
static const struct gpio_dt_spec pin1 = GPIO_DT_SPEC_GET(DEVTYPE_PIN1_NODE, gpios);

uint16_t CONNECTED_DEVICE_ID;

static device_info_t device_info;

int product_info_init(void)
{
	int err;
	int bit0, bit1;

	/* Configure GPIO pins as inputs. */
	if (!gpio_is_ready_dt(&pin0) || !gpio_is_ready_dt(&pin1)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&pin0, GPIO_INPUT);
	if (err) {
		LOG_ERR("Failed to configure pin0, err %d", err);
		return err;
	}

	err = gpio_pin_configure_dt(&pin1, GPIO_INPUT);
	if (err) {
		LOG_ERR("Failed to configure pin1, err %d", err);
		return err;
	}

	/* Read pin values (active-low: 1 = grounded, 0 = floating). */
	bit0 = gpio_pin_get_dt(&pin0);
	bit1 = gpio_pin_get_dt(&pin1);

	if (bit0 < 0 || bit1 < 0) {
		LOG_ERR("Failed to read GPIO pins");
		return -EIO;
	}

	/* Decode device type from pin combination. */
	uint8_t code = (bit0 << 1) | bit1;

	LOG_INF("Device type code: 0x%02X", code);

	switch (code) {
	case 0x00:
		set_device_type(DEVICE_TYPE_GATEWAY);
		break;

	case 0x01:
		set_device_type(DEVICE_TYPE_ANCHOR);
		break;

	case 0x02:
	case 0x03:
		set_device_type(DEVICE_TYPE_SENSOR);
		break;

	default:
		return -EINVAL;
	}

	/* Build serial number: 0x00{device_id_16}00DEADBEEF */
	uint16_t hw_id;

	hwinfo_get_device_id((void *)&hw_id, sizeof(hw_id));
	radio_set_device_id(hw_id);
	set_device_id(hw_id);

	set_serial_number(0xFFFFFFFFFFFFFFFF);

	/* Set initial hop and connected device based on device type. */
	CONNECTED_DEVICE_ID = 0xFFFF;  /* invalid ID by default */
	set_connected_device_id(0xFFFF);  /* invalid ID by default */

	/* Set initial mesh devices count. */
	set_mesh_devices_count(0);

	switch (get_device_type()) {
	case DEVICE_TYPE_GATEWAY:
		set_hop_number(0);
		break;
	case DEVICE_TYPE_ANCHOR:
	case DEVICE_TYPE_SENSOR:
		set_hop_number(0xFF);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

void set_device_type(device_type_t type)
{
	device_info.device_type = type;
}

device_type_t get_device_type(void)
{
	return device_info.device_type;
}

void set_device_id(uint16_t device_id)
{
	device_info.device_id = device_id;
}

uint16_t get_device_id(void)
{
	return device_info.device_id;
}

void set_serial_number(uint64_t serial_num)
{
	device_info.serial_number = serial_num;
}

uint64_t get_serial_number(void)
{
	return device_info.serial_number;
}

void set_hop_number(uint8_t hop_num)
{
	device_info.hop_num = hop_num;
}

uint8_t get_hop_number(void)
{
	return device_info.hop_num;
}

void set_firmware_version(uint16_t firmware_version)
{
	device_info.firmware_version = firmware_version;
}

uint16_t get_firmware_version(void)
{
	return device_info.firmware_version;
}

void set_mesh_devices_count(uint16_t mesh_devices_count)
{
	device_info.mesh_devices_count = mesh_devices_count;
}

uint16_t get_mesh_devices_count(void)
{
	return device_info.mesh_devices_count;
}

void set_connected_device_id(uint16_t device_id)
{
	device_info.connected_device_id = device_id;
}

uint16_t get_connected_device_id(void)
{
	return device_info.connected_device_id;
}

void device_info_update(void)
{
	if (get_device_type() != DEVICE_TYPE_ANCHOR) {
		return;
	}

	if (infra_count == 0) {
		LOG_WRN("No infra devices found, setting hop number to 0xFF and connected device ID to 0xFFFF");
		set_hop_number(0xFF);
		set_connected_device_id(0xFFFF);
		return;
	}

	/* Read all infra entries into a local array. */
	infra_entry_t entry;
	int err = storage_infra_get(0, &entry);
	if (err) {
		LOG_ERR("Failed to read infra storage, err %d", err);
		return;
	}

	uint8_t temp_hop_num = (entry.hop_num < 0xFE) ? entry.hop_num + 1 : 0xFF;

	/* Best entry is now at index 0. */
	if (temp_hop_num != get_hop_number() && get_hop_number() != 0xFF) {
		LOG_INF("Updating hop number from %d to %d", get_hop_number(), temp_hop_num);
		// Send device updated packet to gateway to update hop number and connected device ID
		// Implement Later
	}
	set_hop_number(temp_hop_num);
	set_connected_device_id(entry.device_id);
}

bool is_known_device(uint16_t device_id)
{
	if (get_temp_id() != 0xFFFF && get_temp_id() == device_id) {
		return true;
	}

	for (int i = 0; i < infra_count; i++) {
		if (infra_devices[i].entry.device_id == device_id) {
			infra_devices[i].last_comm_ms = k_uptime_get();
			infra_devices[i].comm_failures = 0;
			infra_devices[i].is_ping_packet_sent = false;
			return true;
		}
	}

	for (int i = 0; i < sensor_count; i++) {
		if (sensor_devices[i].entry.device_id == device_id) {
			sensor_devices[i].last_comm_ms = k_uptime_get();
			sensor_devices[i].comm_failures = 0;
			sensor_devices[i].is_ping_packet_sent = false;
			return true;
		}
	}

	return false;
}

void known_device_update_comm_time(uint16_t device_id, bool is_successful_comm)
{
	if (get_device_type() == DEVICE_TYPE_SENSOR && !is_successful_comm) {
		// Clear Infra Storage and Known Devices
		storage_infra_clear();

		// Send Pair Request because connected device is unreachable, and sensor should send data to gateway through new paired device.
		LOG_WRN("%s ID:%d is unreachable, send PAIR_REQUEST to find new route to gateway", device_type_str(get_device_type()), device_id);
		send_pair_request();
		return;
	} else if (get_device_type() == DEVICE_TYPE_SENSOR) {
		infra_devices[0].last_comm_ms = k_uptime_get();
		infra_devices[0].comm_failures = 0;
		infra_devices[0].is_ping_packet_sent = false;
		return;
	}

	uint64_t now = k_uptime_get();

	for (int i = 0; i < infra_count; i++) {
		if (infra_devices[i].entry.device_id == device_id) {
			if (is_successful_comm) {
				infra_devices[i].comm_failures = 0;
			} else {
				infra_devices[i].comm_failures++;
			}
			infra_devices[i].last_comm_ms = now;
			infra_devices[i].is_ping_packet_sent = false;

			if (infra_devices[i].comm_failures >= MAX_COMM_FAILURES) {
				LOG_WRN("Device ID:%d has %d consecutive communication failures, removing from known devices", device_id, infra_devices[i].comm_failures);
				// Remove device from EEPROM and RAM
				int err = storage_infra_remove(i);
				if (err) {
					LOG_ERR("Failed to remove device from infra storage, err %d", err);
				}

				// Also remove from Route Table and send route update to gateway
			}
			return;
		}
	}

	for (int i = 0; i < sensor_count; i++) {
		if (sensor_devices[i].entry.device_id == device_id) {
			if (is_successful_comm) {
				sensor_devices[i].comm_failures = 0;
			} else {
				sensor_devices[i].comm_failures++;
			}
			sensor_devices[i].last_comm_ms = now;
			sensor_devices[i].is_ping_packet_sent = false;

			if (sensor_devices[i].comm_failures >= MAX_COMM_FAILURES) {
				LOG_WRN("Device ID:%d has %d consecutive communication failures, removing from known devices", device_id, sensor_devices[i].comm_failures);
				// Remove device from EEPROM and RAM
				int err = storage_sensor_remove(i);
				if (err) {
					LOG_ERR("Failed to remove device from sensor storage, err %d", err);
				}
				// Send Device Updated packet to gateway that sensor is removed because it's unreachable


			}
			return;
		}
	}
}


void known_devices_tick(void)
{
	if (get_device_type() != DEVICE_TYPE_GATEWAY) {
		return;
	}
	
	uint64_t now = k_uptime_get();

	for (int i = 0; i < infra_count; i++) {
		if ((now - infra_devices[i].last_comm_ms > PING_TIMEOUT_MS) && !infra_devices[i].is_ping_packet_sent) {
			// Send Ping Device packet to check if device is still reachable
			send_ping_device(infra_devices[i].entry.device_id, infra_devices[i].entry.device_type, 0, STATUS_SUCCESS);
			infra_devices[i].is_ping_packet_sent = true;
		}
	}
}

void ping_known_devices(uint16_t gen_id, uint8_t status)
{
	for (int i = 0; i < infra_count; i++) {
		if (infra_devices[i].entry.hop_num > get_hop_number()) {
			// Send Ping Device packet to check if device is still reachable
			send_ping_device(infra_devices[i].entry.device_id, infra_devices[i].entry.device_type, gen_id, status);
			infra_devices[i].is_ping_packet_sent = true;
		}
	}
}

uint8_t get_hop_num(uint16_t device_id, uint8_t device_type)
{
	if (device_id == 0xFFFF) {
		return 0xFF;
	}

	if (device_type == DEVICE_TYPE_SENSOR) {
		for (int i = 0; i < mesh_count; i++) {
			if (mesh_devices[i].device_id == device_id && mesh_devices[i].device_type == device_type) {
				if (mesh_devices[i].connected_device_id == radio_get_device_id()) {
					return 0;
				}
				for (int j = 0; j < mesh_count; j++) {
					if (mesh_devices[i].connected_device_id == mesh_devices[j].device_id) {
						return mesh_devices[j].hop_num;
					}
				}
				LOG_WRN("Connected device ID %d for device ID %d not found in mesh devices", mesh_devices[i].connected_device_id, device_id);
				return 0xFF;
			}
		}
		LOG_WRN("Device ID %d not found in mesh devices", device_id);
		return 0xFF;
	} else if (device_type == DEVICE_TYPE_ANCHOR) {
		for (int i = 0; i < infra_count; i++) {
			if (infra_devices[i].entry.device_id == device_id && infra_devices[i].entry.device_type == device_type) {
				return infra_devices[i].entry.hop_num;
			}
		}
		return 0xFF;
	} else {
		LOG_WRN("Unknown device type %d in get_hop_num", device_type);
		return 0xFF;
	}
}

void factory_reset(void)
{
	LOG_WRN("Factory Reset.....");

	storage_infra_clear();
	storage_sensor_clear();
	storage_mesh_clear();

	// LOG_WRN("Factory Reset Complete, Rebooting...");
	// k_msleep(500);
	// sys_reboot(SYS_REBOOT_COLD);
}