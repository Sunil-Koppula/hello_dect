/*
 * NVS flash storage backend — drop-in replacement for EEPROM (storage.c).
 *
 * Uses Zephyr NVS on external flash for persistent mesh device tables.
 * Same API as storage.c so the rest of the codebase is unchanged.
 *
 * NVS IDs:
 *   0x10 = infra count
 *   0x11..0x18 = infra entries (index 0–7)
 *   0x20 = sensor count
 *   0x21..0xA0 = sensor entries (index 0–127)
 *   0xB0 = mesh count
 *   0xB1..0xFF+ = mesh entries
 *
 * To remove: delete this file, remove CONFIG_USE_NVS_STORAGE, done.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/logging/log.h>
#include "../storage.h"
#include "../spi_bus.h"
#include "../product_info.h"

LOG_MODULE_REGISTER(nvs_storage, CONFIG_MAIN_LOG_LEVEL);

/* NVS ID layout. */
#define NVS_ID_INFRA_COUNT   0x10
#define NVS_ID_INFRA_BASE    0x11  /* 0x11..0x18 */

#define NVS_ID_SENSOR_COUNT  0x20
#define NVS_ID_SENSOR_BASE   0x21  /* 0x21..0xA0 */

#define NVS_ID_MESH_COUNT    0xB0
#define NVS_ID_MESH_BASE     0xB1  /* 0xB1..onwards */

static struct nvs_fs nvs;
static bool nvs_ready;

/* Cached counts. */
static uint16_t infra_count;
static uint16_t sensor_count;
static uint16_t mesh_count;

static int read_count(uint16_t nvs_id, uint16_t *count)
{
	spi_bus_lock();
	int rc = nvs_read(&nvs, nvs_id, count, sizeof(*count));
	spi_bus_unlock();

	if (rc < 0) {
		*count = 0;
		return 0;  /* not found = 0 entries, not an error */
	}

	return 0;
}

static int write_count(uint16_t nvs_id, uint16_t count)
{
	spi_bus_lock();
	int rc = nvs_write(&nvs, nvs_id, &count, sizeof(count));
	spi_bus_unlock();

	return (rc < 0) ? rc : 0;
}

int storage_init(void)
{
	int err;
	struct flash_pages_info page_info;

	nvs.flash_device = FIXED_PARTITION_DEVICE(storage_partition);
	if (!device_is_ready(nvs.flash_device)) {
		LOG_ERR("Flash device not ready");
		return -ENODEV;
	}

	nvs.offset = FIXED_PARTITION_OFFSET(storage_partition);

	err = flash_get_page_info_by_offs(nvs.flash_device, nvs.offset, &page_info);
	if (err) {
		LOG_ERR("Failed to get flash page info, err %d", err);
		return err;
	}

	nvs.sector_size = page_info.size;
	nvs.sector_count = FIXED_PARTITION_SIZE(storage_partition) / nvs.sector_size;

	err = nvs_mount(&nvs);
	if (err) {
		LOG_ERR("NVS mount failed, err %d", err);
		return err;
	}

	nvs_ready = true;

	read_count(NVS_ID_INFRA_COUNT, &infra_count);
	read_count(NVS_ID_SENSOR_COUNT, &sensor_count);
	read_count(NVS_ID_MESH_COUNT, &mesh_count);

	LOG_INF("NVS storage init: infra=%d sensors=%d mesh=%d",
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

	spi_bus_lock();
	int rc = nvs_write(&nvs, NVS_ID_INFRA_BASE + infra_count,
			   entry, sizeof(*entry));
	spi_bus_unlock();

	if (rc < 0) {
		return rc;
	}

	infra_count++;
	return write_count(NVS_ID_INFRA_COUNT, infra_count);
}

int storage_infra_get(uint16_t index, infra_entry_t *entry)
{
	if (index >= infra_count) {
		return -EINVAL;
	}

	spi_bus_lock();
	int rc = nvs_read(&nvs, NVS_ID_INFRA_BASE + index,
			  entry, sizeof(*entry));
	spi_bus_unlock();

	return (rc < 0) ? rc : 0;
}

int storage_infra_count(void)
{
	return infra_count;
}

int storage_infra_clear(void)
{
	spi_bus_lock();
	for (int i = 0; i < infra_count; i++) {
		nvs_delete(&nvs, NVS_ID_INFRA_BASE + i);
	}
	spi_bus_unlock();

	infra_count = 0;
	return write_count(NVS_ID_INFRA_COUNT, 0);
}

/* ---- Partition 2: Sensors ---- */

int storage_sensor_add(const sensor_entry_t *entry)
{
	if (sensor_count >= MAX_SENSORS) {
		LOG_WRN("Sensor partition full");
		return -ENOMEM;
	}

	spi_bus_lock();
	int rc = nvs_write(&nvs, NVS_ID_SENSOR_BASE + sensor_count,
			   entry, sizeof(*entry));
	spi_bus_unlock();

	if (rc < 0) {
		return rc;
	}

	sensor_count++;
	return write_count(NVS_ID_SENSOR_COUNT, sensor_count);
}

int storage_sensor_get(uint16_t index, sensor_entry_t *entry)
{
	if (index >= sensor_count) {
		return -EINVAL;
	}

	spi_bus_lock();
	int rc = nvs_read(&nvs, NVS_ID_SENSOR_BASE + index,
			  entry, sizeof(*entry));
	spi_bus_unlock();

	return (rc < 0) ? rc : 0;
}

int storage_sensor_count(void)
{
	return sensor_count;
}

int storage_sensor_clear(void)
{
	spi_bus_lock();
	for (int i = 0; i < sensor_count; i++) {
		nvs_delete(&nvs, NVS_ID_SENSOR_BASE + i);
	}
	spi_bus_unlock();

	sensor_count = 0;
	return write_count(NVS_ID_SENSOR_COUNT, 0);
}

/* ---- Partition 3: Mesh network table ---- */

int storage_mesh_add(const mesh_entry_t *entry)
{
	if (mesh_count >= MAX_DEVICES) {
		LOG_WRN("Mesh partition full");
		return -ENOMEM;
	}

	spi_bus_lock();
	int rc = nvs_write(&nvs, NVS_ID_MESH_BASE + mesh_count,
			   entry, sizeof(*entry));
	spi_bus_unlock();

	if (rc < 0) {
		return rc;
	}

	mesh_count++;
	return write_count(NVS_ID_MESH_COUNT, mesh_count);
}

int storage_mesh_get(uint16_t index, mesh_entry_t *entry)
{
	if (index >= mesh_count) {
		return -EINVAL;
	}

	spi_bus_lock();
	int rc = nvs_read(&nvs, NVS_ID_MESH_BASE + index,
			  entry, sizeof(*entry));
	spi_bus_unlock();

	return (rc < 0) ? rc : 0;
}

int storage_mesh_count(void)
{
	return mesh_count;
}

int storage_mesh_clear(void)
{
	spi_bus_lock();
	for (int i = 0; i < mesh_count; i++) {
		nvs_delete(&nvs, NVS_ID_MESH_BASE + i);
	}
	spi_bus_unlock();

	mesh_count = 0;
	return write_count(NVS_ID_MESH_COUNT, 0);
}
