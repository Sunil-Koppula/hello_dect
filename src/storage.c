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
uint8_t infra_count;
uint8_t sensor_count;
uint16_t known_route_count;
static uint16_t mesh_count;

infra_t  infra_devices[MAX_ANCHORS];
sensor_t sensor_devices[MAX_SENSORS];
known_route_t known_route_table[MAX_DEVICES];

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
	infra_count = (uint8_t)count;
	
	err = read_header(STORAGE_PART2_OFFSET, &count);
	if (err) {
		LOG_ERR("Failed to read partition 2 header, err %d", err);
		return err;
	}
	sensor_count = (uint8_t)count;

	err = read_header(STORAGE_PART3_OFFSET, &mesh_count);
	if (err) {
		LOG_ERR("Failed to read partition 3 header, err %d", err);
		return err;
	}

	err = read_header(STORAGE_PART4_OFFSET, &count);
	if (err) {
		LOG_ERR("Failed to read partition 4 header, err %d", err);
		return err;
	}
	known_route_count = count;

	// Update Known infrastructure devices from storage.
	// Use the EEPROM helper directly here — storage_infra_get() now reads
	// from RAM, which is uninitialized until this loop runs.
	LOG_INF("----------Loading %d infra entries from storage----------", infra_count);
	for (uint8_t i = 0; i < infra_count; i++) {
		infra_entry_t entry;
		err = read_entry(STORAGE_PART1_OFFSET, i, &entry, sizeof(entry));
		if (err) {
			LOG_ERR("		Failed to read infra entry %d, err %d", i, err);
			infra_devices[i].entry.device_id = 0xFFFF; /* Mark invalid */
			continue;
		}
		LOG_INF("		Infra entry %d %s ID:%d hop num: %d V: %d RSSI: %d", i, device_type_str(entry.device_type), entry.device_id, entry.hop_num, entry.version, entry.rssi_2);
		infra_devices[i].entry = entry;
		infra_devices[i].last_comm_ms = 0;
		infra_devices[i].comm_failures = 0;
		infra_devices[i].is_ping_packet_sent = false;
	}

	// Update Known Sensors devices from storage. Use the EEPROM helper directly
	// since storage_sensor_get() will read from RAM after this loop initializes it.
	LOG_INF("----------Loading %d sensor entries from storage----------", sensor_count);
	for (uint8_t i = 0; i < sensor_count; i++) {
		sensor_entry_t entry;
		err = read_entry(STORAGE_PART2_OFFSET, i, &entry, sizeof(entry));
		if (err) {
			LOG_ERR("		Failed to read sensor entry %d, err %d", i, err);
			sensor_devices[i].entry.device_id = 0xFFFF; /* Mark invalid */
			continue;
		}
		LOG_INF("		Sensor entry %d %s ID:%d V: %d", i, device_type_str(DEVICE_TYPE_SENSOR), entry.device_id, entry.version);
		sensor_devices[i].entry = entry;
		sensor_devices[i].last_comm_ms = 0;
		sensor_devices[i].comm_failures = 0;
		sensor_devices[i].is_ping_packet_sent = false;
	}

	// Print Mesh entries from storage
	LOG_INF("----------Loading %d mesh entries from storage----------", mesh_count);
	for (uint16_t i = 0; i < mesh_count; i++) {
		mesh_entry_t entry;
		err = storage_mesh_get(i, &entry);
		if (err) {
			LOG_ERR("		Failed to read mesh entry %d, err %d", i, err);
			continue;
		}
		LOG_INF("		Mesh entry %d device %s ID:%d SN: %lld V: %d conn_dev: %d hop_num: %d sensor_count: %d",
			i, device_type_str(entry.device_type), entry.device_id, entry.serial_num, entry.version, entry.connected_device_id, entry.hop_num, entry.sensor_count);
	}

	// Update Known routes from storage
	LOG_INF("----------Loading %d route entries from storage----------", known_route_count);
	for (uint16_t i = 0; i < known_route_count; i++) {
		route_entry_t entry;
		err = storage_route_get(i, &entry);
		if (err) {
			LOG_ERR("		Failed to read route entry %d, err %d", i, err);
			known_route_table[i].device_id = 0xFFFF;
			known_route_table[i].next_device_id = 0xFFFF;
			continue;
		}
		LOG_INF("		Route entry %d device %s ID:%d with %d routes", i, device_type_str(entry.device_type), entry.device_id, entry.route_count);
		for (int j = 0; j < entry.route_count; j++) {
			LOG_INF("			ID: %d Route_len: %d Avg_RSSI: %d", entry.route_info[j].next_hop_id, entry.route_info[j].route_length, entry.route_info[j].avg_rssi_2);
		}
		/* Mirror the best route (index 0 after sort) into the RAM table. */
		known_route_table[i].device_id = entry.device_id;
		known_route_table[i].next_device_id = (entry.route_count > 0) ? entry.route_info[0].next_hop_id : 0xFFFF;
	}
	
	LOG_INF("---------------------------------------------------------");
	LOG_INF("Storage init: infra=%d sensors=%d mesh=%d routes=%d", infra_count, sensor_count, mesh_count, known_route_count);

	return 0;
}

/* ---- Partition 1: Infrastructure ---- */

/* Sort infra_devices[] in place by primary key entry.hop_num ascending
 * (closer to gateway first), secondary key signed entry.rssi_2 descending
 * (stronger signal first). Best entry ends up at index 0.
 *
 * Sorts the RAM mirror only — caller is responsible for flushing the new
 * order to EEPROM via flush_infra_to_eeprom() if persistence is needed. */
static void sort_infra_devices(void)
{
	if (infra_count < 2) {
		return;
	}

	/* Bubble sort directly on infra_devices[] — runtime fields move with
	 * each entry, so we don't need a separate "reorder + carry stats" pass. */
	for (uint8_t i = 0; i < infra_count - 1; i++) {
		for (uint8_t j = 0; j < infra_count - i - 1; j++) {
			bool swap = false;
			if (infra_devices[j].entry.hop_num > infra_devices[j + 1].entry.hop_num) {
				swap = true;
			} else if (infra_devices[j].entry.hop_num == infra_devices[j + 1].entry.hop_num &&
				   infra_devices[j].entry.rssi_2 < infra_devices[j + 1].entry.rssi_2) {
				swap = true;
			}
			if (swap) {
				infra_t temp = infra_devices[j];
				infra_devices[j] = infra_devices[j + 1];
				infra_devices[j + 1] = temp;
			}
		}
	}
}

/* Write all current infra_devices[].entry values to EEPROM (header + entries). */
static int flush_infra_to_eeprom(void)
{
	for (uint8_t i = 0; i < infra_count; i++) {
		int err = write_entry(STORAGE_PART1_OFFSET, i, &infra_devices[i].entry, sizeof(infra_entry_t));
		if (err) {
			LOG_ERR("Failed to write infra entry %d, err %d", i, err);
			return err;
		}
	}
	return write_header(STORAGE_PART1_OFFSET, infra_count);
}

int storage_infra_add(const infra_entry_t *entry)
{
	if (infra_count >= MAX_ANCHORS) {
		LOG_WRN("Infra partition full");
		return -ENOMEM;
	}

	/* Append to RAM table first. */
	infra_devices[infra_count].entry = *entry;
	infra_devices[infra_count].last_comm_ms = 0;
	infra_devices[infra_count].comm_failures = 0;
	infra_devices[infra_count].is_ping_packet_sent = false;
	infra_count++;

	/* Sort RAM, then flush the new order to EEPROM. */
	sort_infra_devices();
	return flush_infra_to_eeprom();
}

int storage_infra_update(uint16_t index, const infra_entry_t *entry)
{
	if (index >= infra_count) {
		return -EINVAL;
	}

	/* Update RAM entry (preserves runtime stats), re-sort, flush to EEPROM. */
	infra_devices[index].entry = *entry;
	sort_infra_devices();
	return flush_infra_to_eeprom();
}

/* Read from RAM mirror — much faster than EEPROM and always in sync with the
 * sorted order. */
int storage_infra_get(uint16_t index, infra_entry_t *entry)
{
	if (index >= infra_count) {
		return -EINVAL;
	}

	*entry = infra_devices[index].entry;
	return 0;
}

int storage_infra_remove(uint16_t index)
{
	if (index >= infra_count) {
		return -EINVAL;
	}

	/* Shift entries after `index` up by one in RAM. */
	for (uint16_t i = index + 1; i < infra_count; i++) {
		infra_devices[i - 1] = infra_devices[i];
	}
	infra_count--;

	/* Order is preserved across a shift, but flush the new layout to EEPROM. */
	return flush_infra_to_eeprom();
}

int storage_infra_count(void)
{
	return infra_count;
}

int storage_infra_clear(void)
{
	infra_count = 0;
	for (int i = 0; i < MAX_ANCHORS; i++) {
		infra_devices[i].entry.device_id = 0xFFFF;
	}
	return write_header(STORAGE_PART1_OFFSET, 0);
}

/* ---- Partition 2: Sensors ---- */

/* Write all current sensor_devices[].entry values to EEPROM (header + entries). */
static int flush_sensors_to_eeprom(void)
{
	for (uint8_t i = 0; i < sensor_count; i++) {
		int err = write_entry(STORAGE_PART2_OFFSET, i, &sensor_devices[i].entry, sizeof(sensor_entry_t));
		if (err) {
			LOG_ERR("Failed to write sensor entry %d, err %d", i, err);
			return err;
		}
	}
	return write_header(STORAGE_PART2_OFFSET, sensor_count);
}

int storage_sensor_add(const sensor_entry_t *entry)
{
	if (sensor_count >= MAX_SENSORS) {
		LOG_WRN("Sensor partition full");
		return -ENOMEM;
	}

	/* Append to RAM table first. */
	sensor_devices[sensor_count].entry = *entry;
	sensor_devices[sensor_count].last_comm_ms = 0;
	sensor_devices[sensor_count].comm_failures = 0;
	sensor_devices[sensor_count].is_ping_packet_sent = false;
	sensor_count++;

	return flush_sensors_to_eeprom();
}

int storage_sensor_update(uint16_t index, const sensor_entry_t *entry)
{
	if (index >= sensor_count) {
		return -EINVAL;
	}

	/* Update RAM entry (preserves runtime stats), then flush to EEPROM. */
	sensor_devices[index].entry = *entry;
	return flush_sensors_to_eeprom();
}

/* Read from RAM mirror — much faster than EEPROM. */
int storage_sensor_get(uint16_t index, sensor_entry_t *entry)
{
	if (index >= sensor_count) {
		return -EINVAL;
	}

	*entry = sensor_devices[index].entry;
	return 0;
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

	/* Shift entries after `index` up by one in RAM. */
	for (uint16_t i = index + 1; i < sensor_count; i++) {
		sensor_devices[i - 1] = sensor_devices[i];
	}
	sensor_count--;

	return flush_sensors_to_eeprom();
}

int storage_sensor_clear(void)
{
	sensor_count = 0;
	for (int i = 0; i < MAX_SENSORS; i++) {
		sensor_devices[i].entry.device_id = 0xFFFF;
	}
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
 *   secondary key: signed avg_rssi_2 descending (stronger signal first).
 * Best route ends up at index 0. */
static void sort_routes(route_entry_t *entry)
{
	if (entry->route_count < 2) {
		return;
	}

	for (int i = 0; i < entry->route_count - 1; i++) {
		for (int j = 0; j < entry->route_count - i - 1; j++) {
			bool swap = false;

			if (entry->route_info[j].route_length > entry->route_info[j + 1].route_length) {
				swap = true;
			} else if (entry->route_info[j].route_length == entry->route_info[j + 1].route_length &&
				   entry->route_info[j].avg_rssi_2 < entry->route_info[j + 1].avg_rssi_2) {
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

static int add_new_route(bool new_entry, uint16_t next_hop_id, uint16_t device_id, uint8_t device_type, uint8_t route_len, int16_t avg_rssi_2, uint16_t device_id_index)
{
	int err = 0;

	if (new_entry) {
		route_entry_t new_route_entry = {
			.device_type = device_type,
			.device_id = device_id,
			.route_count = 1,
			.route_info = { { .next_hop_id = next_hop_id, .route_length = route_len, .avg_rssi_2 = avg_rssi_2 } },
		};
		err = write_entry(STORAGE_PART4_OFFSET, device_id_index, &new_route_entry, sizeof(new_route_entry));
		if (err) {
			return err;
		}

		/* Update in-RAM table: only the best next-hop is mirrored. Since this
		 * is the first (and only) route for the device, it's also the best. */
		known_route_table[known_route_count].device_id = device_id;
		known_route_table[known_route_count].next_device_id = next_hop_id;
		known_route_count++;
		return write_header(STORAGE_PART4_OFFSET, known_route_count);
	}

	/* Update existing device's routes. */
	route_entry_t entry;
	err = read_entry(STORAGE_PART4_OFFSET, device_id_index, &entry, sizeof(entry));
	if (err) {
		return err;
	}

	for (int i = 0; i < entry.route_count; i++) {
		if (entry.route_info[i].next_hop_id == next_hop_id) {
			/* Update existing route, but only if the new one is at least
			 * as short. RSSI keeps the stronger signal (larger signed value). */
			if (entry.route_info[i].route_length >= route_len) {
				if (entry.route_info[i].avg_rssi_2 < avg_rssi_2) {
					entry.route_info[i].avg_rssi_2 = avg_rssi_2;
				}
				entry.route_info[i].route_length = route_len;
			}

			sort_routes(&entry);

			err = write_entry(STORAGE_PART4_OFFSET, device_id_index, &entry, sizeof(entry));
			if (err) {
				return err;
			}

			/* Mirror best (route_info[0]) into the RAM table. */
			known_route_table[device_id_index].next_device_id = entry.route_info[0].next_hop_id;
			return 0;
		}
	}

	/* Route to this next_hop_id not found, add new route if we have space. */
	if (entry.route_count < MAX_ANCHORS) {
		entry.route_info[entry.route_count].next_hop_id = next_hop_id;
		entry.route_info[entry.route_count].route_length = route_len;
		entry.route_info[entry.route_count].avg_rssi_2 = avg_rssi_2;
		entry.route_count++;

		sort_routes(&entry);

		err = write_entry(STORAGE_PART4_OFFSET, device_id_index, &entry, sizeof(entry));
		if (err) {
			return err;
		}

		known_route_table[device_id_index].next_device_id = entry.route_info[0].next_hop_id;
		return 0;
	}
	return -ENOMEM;
}

int storage_route_add(uint16_t next_hop_id, uint16_t device_id, uint8_t device_type, uint8_t route_len, int16_t avg_rssi_2)
{
	/* Find existing device entry; loop naturally terminates at known_route_count
	 * if not found (covers the empty-table case without a special-case shortcut). */
	uint16_t device_id_index;

	for (device_id_index = 0; device_id_index < known_route_count; device_id_index++) {
		if (known_route_table[device_id_index].device_id == device_id) {
			break;
		}
	}

	if (device_id_index == known_route_count) {
		if (known_route_count >= MAX_DEVICES) {
			LOG_WRN("Route partition full, cannot add route for device_id 0x%04X", device_id);
			return -ENOMEM;
		}
		return add_new_route(true, next_hop_id, device_id, device_type, route_len, avg_rssi_2, device_id_index);
	}
	return add_new_route(false, next_hop_id, device_id, device_type, route_len, avg_rssi_2, device_id_index);
}

int storage_route_get(uint16_t index, route_entry_t *entry)
{
	if (index >= known_route_count) {
		return -EINVAL;
	}

	return read_entry(STORAGE_PART4_OFFSET, index, entry, sizeof(*entry));
}

int storage_route_remove(uint16_t device_id, uint16_t next_hop_id)
{
	uint16_t index;
	for (index = 0; index < known_route_count; index++) {
		if (known_route_table[index].device_id == device_id) {
			break;
		}
	}
	if (index == known_route_count) {
		return -EINVAL;
	}

	route_entry_t entry;
	int err = read_entry(STORAGE_PART4_OFFSET, index, &entry, sizeof(entry));
	if (err) {
		return err;
	}

	for (uint8_t i = 0; i < entry.route_count; i++) {
		if (entry.route_info[i].next_hop_id != next_hop_id) {
			continue;
		}

		/* Shift remaining routes up by one. */
		for (uint8_t j = i + 1; j < entry.route_count; j++) {
			entry.route_info[j - 1] = entry.route_info[j];
		}
		entry.route_count--;

		if (entry.route_count == 0) {
			/* Last route for this device removed — shift the whole device
			 * entry out of the partition so it doesn't linger forever. */
			for (uint16_t k = index + 1; k < known_route_count; k++) {
				route_entry_t shifted;
				err = read_entry(STORAGE_PART4_OFFSET, k, &shifted, sizeof(shifted));
				if (err) {
					return err;
				}
				err = write_entry(STORAGE_PART4_OFFSET, k - 1, &shifted, sizeof(shifted));
				if (err) {
					return err;
				}
				known_route_table[k - 1] = known_route_table[k];
			}
			known_route_count--;
			known_route_table[known_route_count].device_id = 0xFFFF;
			known_route_table[known_route_count].next_device_id = 0xFFFF;
			return write_header(STORAGE_PART4_OFFSET, known_route_count);
		}

		err = write_entry(STORAGE_PART4_OFFSET, index, &entry, sizeof(entry));
		if (err) {
			return err;
		}

		/* Mirror best (route_info[0]) into the RAM table. */
		known_route_table[index].next_device_id = entry.route_info[0].next_hop_id;
		return 0;
	}
	return -EINVAL;
}

int storage_route_count(void)
{
	return known_route_count;
}

int storage_route_clear(void)
{
	known_route_count = 0;
	for (int i = 0; i < MAX_DEVICES; i++) {
		known_route_table[i].device_id = 0xFFFF;
		known_route_table[i].next_device_id = 0xFFFF;
	}
	return write_header(STORAGE_PART4_OFFSET, 0);
}