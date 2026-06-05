/*
 * EEPROM storage for mesh network device tables.
 *
 * Partition 1 (0x00000): Infrastructure devices — Gateway + Anchor + Sensor
 * Partition 2 (0x01000): Connected sensors     — Gateway + Anchor
 * Partition 3 (0x02000): Mesh network table    — Gateway only
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/sys/crc.h>
#include "log_color.h"
#include "storage.h"
#include "product_info.h"

LOG_MODULE_REGISTER(storage, CONFIG_STORAGE_LOG_LEVEL);

static const struct device *eeprom_dev = DEVICE_DT_GET(DT_NODELABEL(eeprom));

/* Cached entry counts (avoid reading header every time). */
uint8_t infra_count;
uint8_t sensor_count;
uint16_t mesh_count;

infra_t  infra_devices[MAX_ANCHORS];
sensor_t sensor_devices[MAX_SENSORS];
mesh_t   mesh_devices[MAX_DEVICES];

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

/* Compute CRC32 over an entry struct, excluding its trailing 4-byte crc32 field.
 * Every entry_t in storage.h declares `uint32_t crc32` as its final field. */
static inline uint32_t entry_crc32(const void *entry, size_t entry_size)
{
	return crc32_ieee((const uint8_t *)entry, entry_size - sizeof(uint32_t));
}

/* Returns true if the entry's stored crc32 matches the computed value. */
static inline bool entry_crc_ok(const void *entry, size_t entry_size)
{
	uint32_t stored;
	/* crc32 is the last 4 bytes of the struct; read it without depending on
	 * a specific entry type. */
	memcpy(&stored, (const uint8_t *)entry + entry_size - sizeof(uint32_t),
	       sizeof(stored));
	return entry_crc32(entry, entry_size) == stored;
}

/* Stamp the trailing crc32 field of an entry struct, then write the whole
 * entry to EEPROM. Caller passes a non-const buffer so the crc32 field can
 * be updated in place. */
static int write_entry_with_crc(uint32_t base_offset, uint16_t index,
				void *entry, size_t entry_size)
{
	uint32_t crc = entry_crc32(entry, entry_size);
	memcpy((uint8_t *)entry + entry_size - sizeof(uint32_t), &crc, sizeof(crc));
	return write_entry(base_offset, index, entry, entry_size);
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

	err = read_header(STORAGE_INFRA_OFFSET, &count);
	if (err) {
		LOG_ERR("Failed to read partition 1 header, err %d", err);
		return err;
	}
	infra_count = (uint8_t)count;
	
	err = read_header(STORAGE_SENSOR_OFFSET, &count);
	if (err) {
		LOG_ERR("Failed to read partition 2 header, err %d", err);
		return err;
	}
	sensor_count = (uint8_t)count;

	err = read_header(STORAGE_MESH_OFFSET, &mesh_count);
	if (err) {
		LOG_ERR("Failed to read partition 3 header, err %d", err);
		return err;
	}

	// Update Known infrastructure devices from storage.
	// Use the EEPROM helper directly here — storage_infra_get() now reads
	// from RAM, which is uninitialized until this loop runs.
	LOG_INF_CYAN("Loading %d infra entries from storage", infra_count);
	for (uint8_t i = 0; i < infra_count; i++) {
		infra_entry_t entry;
		err = read_entry(STORAGE_INFRA_OFFSET, i, &entry, sizeof(entry));
		if (err) {
			LOG_ERR("	Failed to read infra entry %d, err %d", i, err);
			infra_devices[i].entry.device_id = 0xFFFF; /* Mark invalid */
			continue;
		}
		if (!entry_crc_ok(&entry, sizeof(entry))) {
			LOG_ERR("	Infra entry %d CRC mismatch, marking invalid", i);
			infra_devices[i].entry.device_id = 0xFFFF;
			continue;
		}
		LOG_INF("	Infra entry %d %s ID:%d hop num: %d V: %d RSSI: %d", i, device_type_str(entry.device_type), entry.device_id, entry.hop_num, entry.version, entry.rssi_2);
		infra_devices[i].entry = entry;
		infra_devices[i].last_comm_ms = 0;
		infra_devices[i].comm_failures = 0;
		infra_devices[i].is_ping_packet_sent = false;
	}

	if (get_device_type() == DEVICE_TYPE_GATEWAY || get_device_type() == DEVICE_TYPE_ANCHOR) {
		// Update Known Sensors devices from storage. Use the EEPROM helper directly
		// since storage_sensor_get() will read from RAM after this loop initializes it.
		LOG_INF_CYAN("Loading %d sensor entries from storage", sensor_count);
		for (uint8_t i = 0; i < sensor_count; i++) {
			sensor_entry_t entry;
			err = read_entry(STORAGE_SENSOR_OFFSET, i, &entry, sizeof(entry));
			if (err) {
				LOG_ERR("	Failed to read sensor entry %d, err %d", i, err);
				sensor_devices[i].entry.device_id = 0xFFFF; /* Mark invalid */
				continue;
			}
			if (!entry_crc_ok(&entry, sizeof(entry))) {
				LOG_ERR("	Sensor entry %d CRC mismatch, marking invalid", i);
				sensor_devices[i].entry.device_id = 0xFFFF;
				continue;
			}
			LOG_INF("	Sensor entry %d %s ID:%d V: %d", i, device_type_str(DEVICE_TYPE_SENSOR), entry.device_id, entry.version);
			sensor_devices[i].entry = entry;
			sensor_devices[i].last_comm_ms = 0;
			sensor_devices[i].comm_failures = 0;
			sensor_devices[i].is_ping_packet_sent = false;
		}
	}

	if (get_device_type() == DEVICE_TYPE_GATEWAY) {
		// Update Mesh devices from storage.
		set_mesh_devices_count(mesh_count);

		// Print Mesh entries from storage
		LOG_INF_CYAN("Loading %d mesh entries from storage", mesh_count);
		for (uint16_t i = 0; i < mesh_count; i++) {
			mesh_entry_t entry;
			err = storage_mesh_get(i, &entry);
			if (err) {
				LOG_ERR("	Failed to read mesh entry %d, err %d", i, err);
				continue;
			}
			LOG_INF("	Mesh entry %d device %s ID:%d SN: %lld V: %d conn_dev: %d hop_num: %d sensor_count: %d",
				i, device_type_str(entry.device_type), entry.device_id, entry.serial_num, entry.version, entry.connected_device_id, entry.hop_num, entry.sensor_count);
			mesh_devices[i].device_type = entry.device_type;
			mesh_devices[i].device_id = entry.device_id;
			mesh_devices[i].serial_num = entry.serial_num;
			mesh_devices[i].connected_device_id = entry.connected_device_id;
			mesh_devices[i].hop_num = entry.hop_num;
		}
	}
	
	LOG_INF("Storage Initialized");

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

/* Write all current infra_devices[].entry values to EEPROM (header + entries).
 * CRC32 is recomputed (covers everything except the crc32 field itself) and
 * stamped into the entry before each write. */
static int flush_infra_to_eeprom(void)
{
	for (uint8_t i = 0; i < infra_count; i++) {
		infra_devices[i].entry.crc32 = entry_crc32(&infra_devices[i].entry, sizeof(infra_entry_t));
		int err = write_entry(STORAGE_INFRA_OFFSET, i, &infra_devices[i].entry, sizeof(infra_entry_t));
		if (err) {
			LOG_ERR("Failed to write infra entry %d, err %d", i, err);
			return err;
		}
	}
	return write_header(STORAGE_INFRA_OFFSET, infra_count);
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
	return write_header(STORAGE_INFRA_OFFSET, 0);
}

/* ---- Partition 2: Sensors ---- */

/* Write all current sensor_devices[].entry values to EEPROM (header + entries).
 * CRC32 is recomputed and stamped before each write. */
static int flush_sensors_to_eeprom(void)
{
	for (uint8_t i = 0; i < sensor_count; i++) {
		sensor_devices[i].entry.crc32 = entry_crc32(&sensor_devices[i].entry, sizeof(sensor_entry_t));
		int err = write_entry(STORAGE_SENSOR_OFFSET, i, &sensor_devices[i].entry, sizeof(sensor_entry_t));
		if (err) {
			LOG_ERR("Failed to write sensor entry %d, err %d", i, err);
			return err;
		}
	}
	return write_header(STORAGE_SENSOR_OFFSET, sensor_count);
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
	return write_header(STORAGE_SENSOR_OFFSET, 0);
}

/* ---- Partition 3: Mesh network table ---- */

int storage_mesh_add(const mesh_entry_t *entry)
{
	if (mesh_count >= MAX_DEVICES) {
		LOG_WRN("Mesh partition full");
		return -ENOMEM;
	}

	mesh_entry_t local = *entry;
	int err = write_entry_with_crc(STORAGE_MESH_OFFSET, mesh_count, &local, sizeof(local));
	if (err) {
		return err;
	}

	mesh_devices[mesh_count].device_type = entry->device_type;
	mesh_devices[mesh_count].device_id = entry->device_id;
	mesh_devices[mesh_count].serial_num = entry->serial_num;
	mesh_devices[mesh_count].connected_device_id = entry->connected_device_id;
	mesh_devices[mesh_count].hop_num = entry->hop_num;

	mesh_count++;
	return write_header(STORAGE_MESH_OFFSET, mesh_count);
}

int storage_mesh_update(uint16_t index, const mesh_entry_t *entry)
{
	if (index >= mesh_count) {
		return -EINVAL;
	}

	mesh_entry_t local = *entry;
	return write_entry_with_crc(STORAGE_MESH_OFFSET, index, &local, sizeof(local));
}

int storage_mesh_get(uint16_t index, mesh_entry_t *entry)
{
	if (index >= mesh_count) {
		return -EINVAL;
	}

	int err = read_entry(STORAGE_MESH_OFFSET, index, entry, sizeof(*entry));
	if (err) {
		return err;
	}
	if (!entry_crc_ok(entry, sizeof(*entry))) {
		LOG_ERR("Mesh entry %d CRC mismatch", index);
		return -EBADMSG;
	}
	return 0;
}

int storage_mesh_count(void)
{
	return mesh_count;
}

int storage_mesh_clear(void)
{
	mesh_count = 0;
	return write_header(STORAGE_MESH_OFFSET, 0);
}

int storage_mesh_remove(uint16_t index)
{
	if (index >= mesh_count) {
		return -EINVAL;
	}

	/* Shift entries after index up by one. The CRC inside each entry is
	 * already valid from when it was last written, so we can reuse it. */
	for (uint16_t i = index + 1; i < mesh_count; i++) {
		mesh_entry_t entry;
		int err = storage_mesh_get(i, &entry);
		if (err) {
			return err;
		}
		err = write_entry(STORAGE_MESH_OFFSET, i - 1, &entry, sizeof(entry));
		if (err) {
			return err;
		}
	}

	mesh_count--;
	return write_header(STORAGE_MESH_OFFSET, mesh_count);
}