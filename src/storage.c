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

	LOG_INF("Storage init: infra=%d sensors=%d mesh=%d",
		infra_count, sensor_count, mesh_count);

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
