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
uint8_t infra_known_device_count;
uint8_t sensor_known_device_count;
static uint16_t mesh_count;
static uint16_t route_count;

infra_known_entry_t  infra_known_devices[MAX_ANCHORS];
sensor_known_entry_t sensor_known_devices[MAX_SENSORS];

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

	uint16_t count;

	if (!device_is_ready(eeprom_dev)) {
		LOG_ERR("EEPROM device not ready");
		return -ENODEV;
	}

	err = read_header(STORAGE_PART1_OFFSET, &count);
	if (err) {
		LOG_ERR("Failed to read partition 1 header, err %d", err);
		return err;
	}
	infra_known_device_count = (uint8_t)count;
	
	err = read_header(STORAGE_PART2_OFFSET, &count);
	if (err) {
		LOG_ERR("Failed to read partition 2 header, err %d", err);
		return err;
	}
	sensor_known_device_count = (uint8_t)count;

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

	// Update Known infrastructure devices from storage
	LOG_INF("----------Loading %d infra entries from storage----------", infra_known_device_count);
	for (uint8_t i = 0; i < infra_known_device_count; i++) {
		infra_entry_t entry;
		err = storage_infra_get(i, &entry);
		if (err) {
			LOG_ERR("		Failed to read infra entry %d, err %d", i, err);
			infra_known_devices[i].device_id = 0xFFFF; /* Mark invalid */
			continue;
		}
		LOG_WRN("		Infra entry %d %s ID:%d from storage", i, device_type_str(entry.device_type), entry.device_id);
		infra_known_devices[i].device_type = entry.device_type;
		infra_known_devices[i].device_id = entry.device_id;
		infra_known_devices[i].last_comm_ms = 0;
		infra_known_devices[i].comm_failures = 0;
		infra_known_devices[i].is_ping_packet_sent = false;
	}

	// Update Known Sensors devices from storage
	LOG_INF("----------Loading %d sensor entries from storage----------", sensor_known_device_count);
	for (uint8_t i = 0; i < sensor_known_device_count; i++) {
		sensor_entry_t entry;
		err = storage_sensor_get(i, &entry);
		if (err) {
			LOG_ERR("		Failed to read sensor entry %d, err %d", i, err);
			sensor_known_devices[i].device_id = 0xFFFF; /* Mark invalid */
			continue;
		}
		LOG_WRN("		Sensor entry %d %s ID:%d from storage", i, device_type_str(DEVICE_TYPE_SENSOR), entry.device_id);
		sensor_known_devices[i].device_id = entry.device_id;
		sensor_known_devices[i].last_comm_ms = 0;
		sensor_known_devices[i].comm_failures = 0;
		sensor_known_devices[i].is_ping_packet_sent = false;
	}

	// Update Known routes from storage
	known_route_count = route_count;
	for (uint16_t i = 0; i < route_count; i++) {
		route_entry_t entry;
		err = storage_route_get(i, &entry);
		if (err) {
			LOG_ERR("Failed to read route entry %d, err %d", i, err);
			known_route_table[i].device_id = 0xFFFF; /* Mark invalid */
			continue;
		}
		LOG_WRN("Loading route entry %d %s ID:%d from storage", i, device_type_str(entry.device_type), entry.device_id);
		known_route_table[i].device_id = entry.device_id;
		for (int j = 0; j < entry.route_count; j++) {
			known_route_table[i].next_device_id[j] = entry.route_info[j].next_hop_id;
			LOG_INF("  next hop %d: ID:%d", j, known_route_table[i].next_device_id[j]);
		}
	}

	LOG_INF("Storage init: infra=%d sensors=%d mesh=%d routes=%d",
		infra_known_device_count, sensor_known_device_count, mesh_count, route_count);

	return 0;
}

/* ---- Partition 1: Infrastructure ---- */

/* Sort infra entries: primary key hop_num ascending (closer to gateway first),
 * secondary key RSSI descending (stronger signal first). Best entry at index 0.
 * Writes the sorted order back to EEPROM. */
static int sort_infra_devices(void)
{
	if (infra_known_device_count < 2) {
		return 0;
	}

	infra_entry_t entry[MAX_ANCHORS];

	for (uint8_t i = 0; i < infra_known_device_count; i++) {
		int err = storage_infra_get(i, &entry[i]);
		if (err) {
			LOG_ERR("Failed to read infra entry %d for sorting, err %d", i, err);
			return err;
		}
	}

	/* Bubble sort: hop_num ascending, then signed RSSI descending. */
	for (uint8_t i = 0; i < infra_known_device_count - 1; i++) {
		for (uint8_t j = 0; j < infra_known_device_count - i - 1; j++) {
			bool swap = false;
			if (entry[j].hop_num > entry[j + 1].hop_num) {
				swap = true;
			} else if (entry[j].hop_num == entry[j + 1].hop_num &&
				   (int16_t)entry[j].rssi_2 < (int16_t)entry[j + 1].rssi_2) {
				swap = true;
			}
			if (swap) {
				infra_entry_t temp = entry[j];
				entry[j] = entry[j + 1];
				entry[j + 1] = temp;
			}
		}
	}

	/* Write the sorted order back to EEPROM. */
	for (uint8_t i = 0; i < infra_known_device_count; i++) {
		int err = write_entry(STORAGE_PART1_OFFSET, i, &entry[i], sizeof(entry[i]));
		if (err) {
			LOG_ERR("Failed to write infra entry %d after sort, err %d", i, err);
			return err;
		}
	}

	/* Mirror the new order into the in-RAM table, preserving each device's
	 * runtime stats (last_comm_ms, comm_failures, is_ping_packet_sent). */
	infra_known_entry_t reordered[MAX_ANCHORS];

	for (uint8_t i = 0; i < infra_known_device_count; i++) {
		bool found = false;
		for (uint8_t j = 0; j < infra_known_device_count; j++) {
			if (infra_known_devices[j].device_id == entry[i].device_id) {
				reordered[i] = infra_known_devices[j];
				found = true;
				break;
			}
		}
		if (!found) {
			/* New device just added — initialize runtime fields. */
			reordered[i].device_type = entry[i].device_type;
			reordered[i].device_id = entry[i].device_id;
			reordered[i].last_comm_ms = 0;
			reordered[i].comm_failures = 0;
			reordered[i].is_ping_packet_sent = false;
		}
	}

	for (uint8_t i = 0; i < infra_known_device_count; i++) {
		infra_known_devices[i] = reordered[i];
	}

	return 0;
}

int storage_infra_add(const infra_entry_t *entry)
{
	if (infra_known_device_count >= MAX_ANCHORS) {
		LOG_WRN("Infra partition full");
		return -ENOMEM;
	}

	int err = write_entry(STORAGE_PART1_OFFSET, infra_known_device_count, entry, sizeof(*entry));
	if (err) {
		return err;
	}

	infra_known_device_count++;

	err = write_header(STORAGE_PART1_OFFSET, infra_known_device_count);
	if (err) {
		return err;
	}

	return sort_infra_devices();
}

int storage_infra_update(uint16_t index, const infra_entry_t *entry)
{
	if (index >= infra_known_device_count) {
		return -EINVAL;
	}

	int err = write_entry(STORAGE_PART1_OFFSET, index, entry, sizeof(*entry));
	if (err) {
		return err;
	}

	return sort_infra_devices();
}

int storage_infra_get(uint16_t index, infra_entry_t *entry)
{
	if (index >= infra_known_device_count) {
		return -EINVAL;
	}

	return read_entry(STORAGE_PART1_OFFSET, index, entry, sizeof(*entry));
}

int storage_infra_remove(uint16_t index)
{
	if (index >= infra_known_device_count) {
		return -EINVAL;
	}

	/* Shift entries after index up by one. */
	for (uint16_t i = index + 1; i < infra_known_device_count; i++) {
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

	infra_known_device_count--;
	int err = write_header(STORAGE_PART1_OFFSET, infra_known_device_count);
	if (err) {
		return err;
	}
	return sort_infra_devices();
}

int storage_infra_count(void)
{
	return infra_known_device_count;
}

int storage_infra_clear(void)
{
	infra_known_device_count = 0;
	return write_header(STORAGE_PART1_OFFSET, 0);
}

/* ---- Partition 2: Sensors ---- */

int storage_sensor_add(const sensor_entry_t *entry)
{
	if (sensor_known_device_count >= MAX_SENSORS) {
		LOG_WRN("Sensor partition full");
		return -ENOMEM;
	}

	int err = write_entry(STORAGE_PART2_OFFSET, sensor_known_device_count, entry, sizeof(*entry));

	if (err) {
		return err;
	}

	sensor_known_device_count++;
	err = write_header(STORAGE_PART2_OFFSET, sensor_known_device_count);
	if (err) {
		return err;
	}

	// Mirror to in-RAM table
	sensor_known_devices[sensor_known_device_count - 1].device_id = entry->device_id;
	sensor_known_devices[sensor_known_device_count - 1].last_comm_ms = 0;
	sensor_known_devices[sensor_known_device_count - 1].comm_failures = 0;
	sensor_known_devices[sensor_known_device_count - 1].is_ping_packet_sent = false;
	return 0;
}

int storage_sensor_update(uint16_t index, const sensor_entry_t *entry)
{
	if (index >= sensor_known_device_count) {
		return -EINVAL;
	}

	return write_entry(STORAGE_PART2_OFFSET, index, entry, sizeof(*entry));
}

int storage_sensor_get(uint16_t index, sensor_entry_t *entry)
{
	if (index >= sensor_known_device_count) {
		return -EINVAL;
	}

	return read_entry(STORAGE_PART2_OFFSET, index, entry, sizeof(*entry));
}

int storage_sensor_count(void)
{
	return sensor_known_device_count;
}

int storage_sensor_remove(uint16_t index)
{
	if (index >= sensor_known_device_count) {
		return -EINVAL;
	}

	/* Shift entries after index up by one. */
	for (uint16_t i = index + 1; i < sensor_known_device_count; i++) {
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

	sensor_known_device_count--;
	int err = write_header(STORAGE_PART2_OFFSET, sensor_known_device_count);
	if (err) {
		return err;
	}

	// Mirror the shift in the in-RAM table
	for (uint8_t i = index; i < sensor_known_device_count; i++) {
		sensor_known_devices[i] = sensor_known_devices[i + 1];
	}
	return 0;
}

int storage_sensor_clear(void)
{
	sensor_known_device_count = 0;
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