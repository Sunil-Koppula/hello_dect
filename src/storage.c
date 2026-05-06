/*
 * EEPROM storage for mesh network device tables.
 *
 * Partition 1 (0x00000): Infrastructure devices — Gateway + Anchor + Sensor
 * Partition 2 (0x01000): Connected sensors     — Gateway + Anchor
 * Partition 3 (0x02000): Mesh network table    — Gateway only
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/logging/log.h>
#include "storage.h"
#include "product_info.h"

LOG_MODULE_REGISTER(storage, CONFIG_STORAGE_LOG_LEVEL);

static const struct device *eeprom_dev = DEVICE_DT_GET(DT_NODELABEL(eeprom));

/* Cached entry counts (avoid reading header every time). */
static uint16_t infra_count;
static uint16_t sensor_count;
static uint16_t mesh_count;
static uint16_t route_count;

/* ---- Internal helpers ---- */

static int read_header(uint32_t base_offset, uint16_t *count)
{
	uint8_t hdr[STORAGE_HEADER_SIZE];
	int err;

	err = eeprom_read(eeprom_dev, base_offset, hdr, sizeof(hdr));
	if (err) {
		return err;
	}

	if (hdr[0] != STORAGE_MAGIC_0 || hdr[1] != STORAGE_MAGIC_1) {
		*count = 0;
		return 0;
	}

	*count = (uint16_t)(hdr[2] | (hdr[3] << 8));
	return 0;
}

static int write_header(uint32_t base_offset, uint16_t count)
{
	uint8_t hdr[STORAGE_HEADER_SIZE] = {
		STORAGE_MAGIC_0,
		STORAGE_MAGIC_1,
		(uint8_t)(count & 0xFF),
		(uint8_t)(count >> 8),
	};

	return eeprom_write(eeprom_dev, base_offset, hdr, sizeof(hdr));
}

static int read_entry(uint32_t base_offset, uint16_t index, void *entry, size_t entry_size)
{
	uint32_t addr = base_offset + STORAGE_HEADER_SIZE + (index * entry_size);

	return eeprom_read(eeprom_dev, addr, entry, entry_size);
}

static int write_entry(uint32_t base_offset, uint16_t index, const void *entry, size_t entry_size)
{
	uint32_t addr = base_offset + STORAGE_HEADER_SIZE + (index * entry_size);

	return eeprom_write(eeprom_dev, addr, entry, entry_size);
}

/* ---- Public API ---- */

int storage_init(void)
{
	int err;

	if (!device_is_ready(eeprom_dev)) {
		LOG_ERR("EEPROM device not ready");
		return -ENODEV;
	}

	err = read_header(STORAGE_PART1_OFFSET, &infra_count);
	if (err) {
		LOG_ERR("Failed to read partition 1 header, err %d", err);
		return err;
	}

	err = read_header(STORAGE_PART2_OFFSET, &sensor_count);
	if (err) {
		LOG_ERR("Failed to read partition 2 header, err %d", err);
		return err;
	}

	err = read_header(STORAGE_PART3_OFFSET, &mesh_count);
	if (err) {
		LOG_ERR("Failed to read partition 3 header, err %d", err);
		return err;
	}

	err = read_header(STORAGE_PART4_OFFSET, &route_count);
	if (err) {
		LOG_ERR("Failed to read partition 4 header, err %d", err);
		return err;
	}

	// Update Known routes from storage
	known_route_count = route_count;
	for (uint16_t i = 0; i < route_count; i++) {
		LOG_WRN("Loading route entry %d from storage", i);
		route_entry_t entry;
		err = storage_route_get(i, &entry);
		if (err) {
			LOG_ERR("Failed to read route entry %d, err %d", i, err);
			known_route_table[i].device_id = 0xFFFF; /* Mark invalid */
			continue;
		}
		for (int j = 0; j < entry.route_count; j++) {
			known_route_table[i].next_device_id[j] = entry.route_info[j].next_hop_id;
		}
	}

	LOG_INF("Storage init: infra=%d sensors=%d mesh=%d routes=%d",
		infra_count, sensor_count, mesh_count, route_count);

	return 0;
}

/* ---- Partition 1: Infrastructure ---- */

int storage_infra_add(const infra_entry_t *entry)
{
	if (infra_count >= MAX_ANCHORS) {
		LOG_WRN("Infra partition full");
		return -ENOMEM;
	}

	int err = write_entry(STORAGE_PART1_OFFSET, infra_count, entry, sizeof(*entry));

	if (err) {
		return err;
	}

	infra_count++;
	return write_header(STORAGE_PART1_OFFSET, infra_count);
}

int storage_infra_update(uint16_t index, const infra_entry_t *entry)
{
	if (index >= infra_count) {
		return -EINVAL;
	}

	return write_entry(STORAGE_PART1_OFFSET, index, entry, sizeof(*entry));
}

int storage_infra_get(uint16_t index, infra_entry_t *entry)
{
	if (index >= infra_count) {
		return -EINVAL;
	}

	return read_entry(STORAGE_PART1_OFFSET, index, entry, sizeof(*entry));
}

int storage_infra_remove(uint16_t index)
{
	if (index >= infra_count) {
		return -EINVAL;
	}

	/* Shift entries after index up by one. */
	for (uint16_t i = index + 1; i < infra_count; i++) {
		infra_entry_t entry;
		int err = storage_infra_get(i, &entry);
		if (err) {
			return err;
		}
		err = write_entry(STORAGE_PART1_OFFSET, i - 1, &entry, sizeof(entry));
		if (err) {
			return err;
		}
	}

	infra_count--;
	return write_header(STORAGE_PART1_OFFSET, infra_count);
}

int storage_infra_count(void)
{
	return infra_count;
}

int storage_infra_clear(void)
{
	infra_count = 0;
	return write_header(STORAGE_PART1_OFFSET, 0);
}

/* ---- Partition 2: Sensors ---- */

int storage_sensor_add(const sensor_entry_t *entry)
{
	if (sensor_count >= MAX_SENSORS) {
		LOG_WRN("Sensor partition full");
		return -ENOMEM;
	}

	int err = write_entry(STORAGE_PART2_OFFSET, sensor_count, entry, sizeof(*entry));

	if (err) {
		return err;
	}

	sensor_count++;
	return write_header(STORAGE_PART2_OFFSET, sensor_count);
}

int storage_sensor_update(uint16_t index, const sensor_entry_t *entry)
{
	if (index >= sensor_count) {
		return -EINVAL;
	}

	return write_entry(STORAGE_PART2_OFFSET, index, entry, sizeof(*entry));
}

int storage_sensor_get(uint16_t index, sensor_entry_t *entry)
{
	if (index >= sensor_count) {
		return -EINVAL;
	}

	return read_entry(STORAGE_PART2_OFFSET, index, entry, sizeof(*entry));
}

int storage_sensor_count(void)
{
	return sensor_count;
}

int storage_sensor_remove(uint16_t index)
{
	if (index >= sensor_count) {
		return -EINVAL;
	}

	/* Shift entries after index up by one. */
	for (uint16_t i = index + 1; i < sensor_count; i++) {
		sensor_entry_t entry;
		int err = storage_sensor_get(i, &entry);
		if (err) {
			return err;
		}
		err = write_entry(STORAGE_PART2_OFFSET, i - 1, &entry, sizeof(entry));
		if (err) {
			return err;
		}
	}

	sensor_count--;
	return write_header(STORAGE_PART2_OFFSET, sensor_count);
}

int storage_sensor_clear(void)
{
	sensor_count = 0;
	return write_header(STORAGE_PART2_OFFSET, 0);
}

/* ---- Partition 3: Mesh network table ---- */

int storage_mesh_add(const mesh_entry_t *entry)
{
	if (mesh_count >= MAX_DEVICES) {
		LOG_WRN("Mesh partition full");
		return -ENOMEM;
	}

	int err = write_entry(STORAGE_PART3_OFFSET, mesh_count, entry, sizeof(*entry));

	if (err) {
		return err;
	}

	mesh_count++;
	return write_header(STORAGE_PART3_OFFSET, mesh_count);
}

int storage_mesh_update(uint16_t index, const mesh_entry_t *entry)
{
	if (index >= mesh_count) {
		return -EINVAL;
	}

	return write_entry(STORAGE_PART3_OFFSET, index, entry, sizeof(*entry));
}

int storage_mesh_get(uint16_t index, mesh_entry_t *entry)
{
	if (index >= mesh_count) {
		return -EINVAL;
	}

	return read_entry(STORAGE_PART3_OFFSET, index, entry, sizeof(*entry));
}

int storage_mesh_count(void)
{
	return mesh_count;
}

int storage_mesh_clear(void)
{
	mesh_count = 0;
	return write_header(STORAGE_PART3_OFFSET, 0);
}

int storage_mesh_remove(uint16_t index)
{
	if (index >= mesh_count) {
		return -EINVAL;
	}

	/* Shift entries after index up by one. */
	for (uint16_t i = index + 1; i < mesh_count; i++) {
		mesh_entry_t entry;
		int err = storage_mesh_get(i, &entry);
		if (err) {
			return err;
		}
		err = write_entry(STORAGE_PART3_OFFSET, i - 1, &entry, sizeof(entry));
		if (err) {
			return err;
		}
	}

	mesh_count--;
	return write_header(STORAGE_PART3_OFFSET, mesh_count);
}

/* ---- Partition 4: Routing table for mesh network ---- */

/* Sort entry->route_info[0..route_count-1] in place:
 *   primary key: route_length ascending (shorter = better),
 *   secondary key: avg_rssi_2 ascending (smaller = stronger).
 * Best route ends up at index 0. */
static void sort_routes(route_entry_t *entry)
{
	for (int i = 0; i < entry->route_count - 1; i++) {
		for (int j = 0; j < entry->route_count - i - 1; j++) {
			bool swap = false;

			if (entry->route_info[j].route_length > entry->route_info[j + 1].route_length) {
				swap = true;
			} else if (entry->route_info[j].route_length == entry->route_info[j + 1].route_length &&
				   entry->route_info[j].avg_rssi_2 > entry->route_info[j + 1].avg_rssi_2) {
				swap = true;
			}

			if (swap) {
				route_t tmp = entry->route_info[j];
				entry->route_info[j] = entry->route_info[j + 1];
				entry->route_info[j + 1] = tmp;
			}
		}
	}
}

static int add_new_route(bool new_entry, uint16_t dst_id, uint16_t device_id, uint8_t device_type, uint8_t route_len, int16_t avg_rssi_2, uint16_t device_id_index) {
	int err = 0;

	if (new_entry) {
		route_entry_t new_entry = {
			.device_type = device_type,
			.device_id = device_id,
			.route_count = 1,
			.route_info = { { .next_hop_id = dst_id, .route_length = route_len, .avg_rssi_2 = avg_rssi_2 } },
		};
		err = write_entry(STORAGE_PART4_OFFSET, device_id_index, &new_entry, sizeof(new_entry));
		if (err) {
			return err;
		}
		// also update known route table in memory
		known_route_table[known_route_count].device_id = device_id;
		known_route_table[known_route_count++].next_device_id[0] = dst_id;
		route_count++;
		return write_header(STORAGE_PART4_OFFSET, route_count);
	} else {
		route_entry_t entry;
		err = read_entry(STORAGE_PART4_OFFSET, device_id_index, &entry, sizeof(entry));
		if (err) {
			return err;
		}
		for (int i = 0; i < entry.route_count; i++) {
			if (entry.route_info[i].next_hop_id == dst_id) {
				// Update existing route
				if (entry.route_info[i].route_length >= route_len) {
					if (entry.route_info[i].avg_rssi_2 > avg_rssi_2) {
						entry.route_info[i].avg_rssi_2 = avg_rssi_2;
					}
					entry.route_info[i].route_length = route_len;
				}
				err = write_entry(STORAGE_PART4_OFFSET, device_id_index, &entry, sizeof(entry));
				if (err) {
					return err;
				}
				// Sort routes by route length (and then by RSSI if lengths are equal)
				sort_routes(&entry);
				// Update in EEPROM after sorting
				err = write_entry(STORAGE_PART4_OFFSET, device_id_index, &entry, sizeof(entry));
				if (err) {
					return err;
				}
				// also update known route table in memory
				for (int j = 0; j < entry.route_count; j++) {
					known_route_table[device_id_index].next_device_id[j] = entry.route_info[j].next_hop_id;
				}
				return err;
			}
		}
		// Route to this dst_id not found, add new route if we have space
		if (entry.route_count < MAX_ANCHORS) {
			entry.route_info[entry.route_count].next_hop_id = dst_id;
			entry.route_info[entry.route_count].route_length = route_len;
			entry.route_info[entry.route_count].avg_rssi_2 = avg_rssi_2;
			entry.route_count++;
			err = write_entry(STORAGE_PART4_OFFSET, device_id_index, &entry, sizeof(entry));
			if (err) {
				return err;
			}
			// Sort routes by route length (and then by RSSI if lengths are equal)
			sort_routes(&entry);
			// Update in EEPROM after sorting
			err = write_entry(STORAGE_PART4_OFFSET, device_id_index, &entry, sizeof(entry));
			if (err) {
				return err;
			}
			// also update known route table in memory
			for (int j = 0; j < entry.route_count; j++) {
				known_route_table[device_id_index].next_device_id[j] = entry.route_info[j].next_hop_id;
			}
			return err;
		}
		return -ENOMEM;
	}
}

int storage_route_add(uint16_t dst_id, uint16_t device_id, uint8_t device_type, uint8_t route_len, int16_t avg_rssi_2)
{
	if (route_count == 0) {
		// No existing routes, add new entry
		return add_new_route(true, dst_id, device_id, device_type, route_len, avg_rssi_2, 0);
	}

	uint16_t device_id_index = 0;

	for (device_id_index = 0; device_id_index < known_route_count; device_id_index++) {
		if (known_route_table[device_id_index].device_id == device_id) {
			break;
		}
	}
	// If we didn't find an existing entry for this device, add a new one
	if (device_id_index == known_route_count && known_route_count < MAX_DEVICES) {
		// Add new entry
		return add_new_route(true, dst_id, device_id, device_type, route_len, avg_rssi_2, device_id_index);
	} else if (device_id_index == known_route_count) {
		// No existing entry and no space for new entry
		LOG_WRN("Route partition full, cannot add route for device_id 0x%04X", device_id);
		return -ENOMEM;
	}
	// We found an existing entry for this device, add/update route in that entry
	return add_new_route(false, dst_id, device_id, device_type, route_len, avg_rssi_2, device_id_index);

}

int storage_route_get(uint16_t index, route_entry_t *entry)
{
	if (index >= route_count) {
		return -EINVAL;
	}

	return read_entry(STORAGE_PART4_OFFSET, index, entry, sizeof(*entry));
}

int storage_route_remove(uint16_t device_id, uint16_t dst_id)
{
	uint16_t index = 0;
	for (index = 0; index < known_route_count; index++) {
		if (known_route_table[index].device_id == device_id) {
			break;
		}
	}
	if (index == known_route_count) {
		return -EINVAL;
	}

	/* Shift entries after index up by one. */
	route_entry_t entry;
	int err = read_entry(STORAGE_PART4_OFFSET, index, &entry, sizeof(entry));
	if (err) {
		return err;
	}
	for (uint8_t i = 0; i < entry.route_count; i++) {
		if (entry.route_info[i].next_hop_id == dst_id) {
			// Shift remaining routes up by one
			for (uint8_t j = i + 1; j < entry.route_count; j++) {
				entry.route_info[j - 1] = entry.route_info[j];
			}
			entry.route_count--;
			err = write_entry(STORAGE_PART4_OFFSET, index, &entry, sizeof(entry));
			if (err) {
				return err;
			}
			if (entry.route_count == 0) {
				for (int j = 0; j < MAX_ANCHORS; j++) {
					known_route_table[index].next_device_id[j] = 0xFFFF; /* Mark invalid */
				}
				return err;
			}
			// also update known route table in memory
			for (int j = 0; j < entry.route_count; j++) {
				known_route_table[index].next_device_id[j] = entry.route_info[j].next_hop_id;
			}
			return err;
		}
	}
	return -EINVAL;
}

int storage_route_count(void)
{
	return route_count;
}

int storage_route_clear(void)
{
	route_count = 0;
	return write_header(STORAGE_PART4_OFFSET, 0);
}