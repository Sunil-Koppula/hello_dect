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
#include <zephyr/logging/log.h>
#include "main_sub.h"
#include "product_info.h"
#include "tracker.h"
#include "radio.h"
#include "mesh.h"
#include "storage.h"

LOG_MODULE_REGISTER(product_info, CONFIG_PRODUCT_INFO_LOG_LEVEL);

#define DEVTYPE_PIN0_NODE DT_NODELABEL(devtype_pin0)
#define DEVTYPE_PIN1_NODE DT_NODELABEL(devtype_pin1)

static const struct gpio_dt_spec pin0 = GPIO_DT_SPEC_GET(DEVTYPE_PIN0_NODE, gpios);
static const struct gpio_dt_spec pin1 = GPIO_DT_SPEC_GET(DEVTYPE_PIN1_NODE, gpios);

device_type_t PRODUCT_DEVICE_TYPE;
uint64_t PRODUCT_SERIAL_NUMBER;
uint8_t PRODUCT_HOP_NUMBER;
uint16_t PRODUCT_CONNECTED_DEVICE_ID;

known_device_t known_devices[MAX_KNOWN_DEVICES];
uint8_t known_device_count;

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

	switch (code) {
	case 0x00:
		PRODUCT_DEVICE_TYPE = DEVICE_TYPE_GATEWAY;
		break;
	case 0x01:
		PRODUCT_DEVICE_TYPE = DEVICE_TYPE_ANCHOR;
		break;
	case 0x02:
	case 0x03:
	default:
		PRODUCT_DEVICE_TYPE = DEVICE_TYPE_SENSOR;
		break;
	}

	/* Build serial number: 0x00{device_id_16}00DEADBEEF */
	uint16_t hw_id;

	hwinfo_get_device_id((void *)&hw_id, sizeof(hw_id));
	PRODUCT_SERIAL_NUMBER = ((uint64_t)hw_id << 40) | 0x00DEADBEEFULL;

	/* Set initial hop and connected device based on device type. */
	PRODUCT_CONNECTED_DEVICE_ID = 0xFFFF;  /* invalid ID by default */

	switch (PRODUCT_DEVICE_TYPE) {
	case DEVICE_TYPE_GATEWAY:
		PRODUCT_HOP_NUMBER = 0;
		break;
	case DEVICE_TYPE_SENSOR:
		PRODUCT_HOP_NUMBER = 0xFF;  /* sensors don't have hop */
		break;
	default:
		PRODUCT_HOP_NUMBER = 0xFF;  /* anchor: updated after pairing */
		break;
	}

	LOG_INF("Device type: %s, SN: 0x%016llx, Hop: %d",
		device_type_str(PRODUCT_DEVICE_TYPE),
		PRODUCT_SERIAL_NUMBER, PRODUCT_HOP_NUMBER);

	return 0;
}

void product_info_update_hop(void)
{
	if (PRODUCT_DEVICE_TYPE != DEVICE_TYPE_ANCHOR) {
		return;
	}

	int infra = storage_infra_count();

	if (infra == 0) {
		PRODUCT_HOP_NUMBER = 0xFF;
		PRODUCT_CONNECTED_DEVICE_ID = 0;
		return;
	}

	/* Read all infra entries into a local array. */
	infra_entry_t entries[MAX_ANCHORS];
	int count = 0;

	for (int i = 0; i < infra && i < MAX_ANCHORS; i++) {
		if (storage_infra_get(i, &entries[count]) == 0) {
			count++;
		}
	}

	/* Sort by hop ascending, then RSSI descending (better signal). */
	for (int i = 0; i < count - 1; i++) {
		for (int j = 0; j < count - i - 1; j++) {
			bool swap = false;

			if (entries[j].hop_num > entries[j + 1].hop_num) {
				swap = true;
			} else if (entries[j].hop_num == entries[j + 1].hop_num &&
				   entries[j].rssi_2 < entries[j + 1].rssi_2) {
				swap = true;
			}

			if (swap) {
				infra_entry_t tmp = entries[j];
				entries[j] = entries[j + 1];
				entries[j + 1] = tmp;
			}
		}
	}

	/* Write sorted entries back to storage. */
	storage_infra_clear();

	for (int i = 0; i < count; i++) {
		storage_infra_add(&entries[i]);
	}

	/* Best entry is now at index 0. */
	PRODUCT_HOP_NUMBER = (entries[0].hop_num < 0xFE) ? entries[0].hop_num + 1 : 0xFF;
	PRODUCT_CONNECTED_DEVICE_ID = entries[0].device_id;

	LOG_INF("Anchor hop updated: %d, connected to %s ID:%d (hop: %d), infra sorted",
		PRODUCT_HOP_NUMBER,
		device_type_str(entries[0].device_type),
		PRODUCT_CONNECTED_DEVICE_ID,
		entries[0].hop_num);
}

void update_known_devices(void)
{
	uint8_t count = 0;

	for (int i = 0; i < storage_infra_count() && count < MAX_KNOWN_DEVICES; i++) {
		infra_entry_t entry;
		if (storage_infra_get(i, &entry) == 0) {
			known_devices[count].device_type = entry.device_type;
			known_devices[count++].device_id = entry.device_id;
		}
	}

	for (int i = 0; i < storage_sensor_count() && count < MAX_KNOWN_DEVICES; i++) {
		sensor_entry_t entry;
		if (storage_sensor_get(i, &entry) == 0) {
			known_devices[count].device_type = DEVICE_TYPE_SENSOR;
			known_devices[count++].device_id = entry.device_id;
		}
	}

	known_device_count = count;

}

bool is_known_device(uint16_t device_id)
{
	for (int i = 0; i < known_device_count; i++) {
		if (known_devices[i].device_id == device_id) {
			known_devices[i].last_comm_ms = k_uptime_get();
			known_devices[i].comm_failures = 0;
			known_devices[i].is_ping_packet_sent = false;
			return true;
		}
	}
	return false;
}

int known_device_idx(uint16_t device_id)
{
	for (int i = 0; i < known_device_count; i++) {
		if (known_devices[i].device_id == device_id) {
			return i;
		}
	}
	return -1;
}

void known_device_update_comm_time(uint16_t device_id, bool is_successful_comm)
{
	if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_SENSOR && !is_successful_comm) {
		// Clear Infra Storage and Known Devices
		storage_infra_clear();
		known_device_count = 0;

		// Send Pair Request because connected device is unreachable, and sensor should send data to gateway through new paired device.
		uint8_t tid = tracker_next_id();
		tracker_add(device_id, radio_get_device_id(), tid, PACKET_PAIR_REQUEST, PAIR_TIMEOUT_MS, PAIR_MAX_RETRIES, NULL, 0);
		LOG_INF("%s ID:%d is unreachable, sending PAIR_REQUEST to find new route to gateway", device_type_str(PRODUCT_DEVICE_TYPE), device_id);
		send_pair_request(tid);
		return;
	}
	uint64_t now = k_uptime_get();

	for (int i = 0; i < known_device_count; i++) {
		if (known_devices[i].device_id == device_id) {
			if (is_successful_comm) {
				known_devices[i].comm_failures = 0;
			} else {
				known_devices[i].comm_failures++;
			}
			known_devices[i].last_comm_ms = now;
			known_devices[i].is_ping_packet_sent = false;

			if (known_devices[i].comm_failures >= MAX_COMM_FAILURES) {
				LOG_WRN("Device ID:%d has %d consecutive communication failures, removing from known devices", device_id, known_devices[i].comm_failures);
				// Remove device from known devices list
				for (int j = i; j < known_device_count - 1; j++) {
					known_devices[j] = known_devices[j + 1];
				}
				known_device_count--;

				// If it's an infra device, also remove from infra storage
				for (int j = 0; j < storage_infra_count(); j++) {
					infra_entry_t entry;
					if (storage_infra_get(j, &entry) == 0 && entry.device_id == device_id) {
						storage_infra_remove(j);
						LOG_INF("Removed %s ID:%d from infra storage due to communication failures", device_type_str(entry.device_type), device_id);
						break;
					}
				}
			}
			// Implement Device Removed logic to update route info

			break;
		}
	}
}


void known_devices_tick(void)
{
	uint64_t now = k_uptime_get();

	for (int i = 0; i < known_device_count; i++) {
		if ((now - known_devices[i].last_comm_ms > PING_TIMEOUT_MS) && (known_devices[i].device_type != DEVICE_TYPE_SENSOR) && !known_devices[i].is_ping_packet_sent) {
			// Send Ping Device packet to check if device is still reachable
			uint8_t tid = tracker_next_id();
			tracker_add(known_devices[i].device_id, radio_get_device_id(), tid, PACKET_PING_DEVICE, 500, 5, NULL, 0);
			LOG_INF("Sending PING_DEVICE to known device ID:%d to check connectivity", known_devices[i].device_id);
			send_ping_device(known_devices[i].device_id, tid, STATUS_SUCCESS);
			known_devices[i].is_ping_packet_sent = true;
		}
	}
}

void ping_known_devices(void)
{
	for (int i = 0; i < known_device_count; i++) {
		if (known_devices[i].device_type != DEVICE_TYPE_SENSOR) {  /* Only ping non-sensor devices since sensors don't store hop/RSSI and are less critical to keep updated */
			uint8_t tid = tracker_next_id();
			tracker_add(known_devices[i].device_id, radio_get_device_id(), tid, PACKET_PING_DEVICE, 500, 5, NULL, 0);
			LOG_INF("Pinging known device ID:%d at startup to check connectivity", known_devices[i].device_id);
			send_ping_device(known_devices[i].device_id, tid, STATUS_SUCCESS);
			known_devices[i].is_ping_packet_sent = true;
		}
	}
}