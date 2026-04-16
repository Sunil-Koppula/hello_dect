/*
 * PSRAM-backed ring buffer queue for DECT NR+ mesh network.
 *
 * Stores queue data in external PSRAM via SPI, keeping only
 * head/tail/count metadata in internal RAM (~12 bytes per queue).
 *
 * This allows massive queue depths (8MB PSRAM) with minimal
 * internal RAM usage.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "psram_queue.h"
#include "psram.h"

LOG_MODULE_REGISTER(psram_queue, CONFIG_PSRAM_QUEUE_LOG_LEVEL);

int psram_queue_init(struct psram_queue *q, uint32_t base_addr,
		     uint32_t region_size, uint16_t item_size)
{
	if (item_size == 0 || region_size < item_size) {
		LOG_ERR("Invalid psram_queue params: region=%d item=%d",
			region_size, item_size);
		return -EINVAL;
	}

	if (base_addr + region_size > PSRAM_SIZE) {
		LOG_ERR("PSRAM queue region exceeds PSRAM size");
		return -EINVAL;
	}

	q->base_addr = base_addr;
	q->region_size = region_size;
	q->item_size = item_size;
	q->capacity = region_size / item_size;
	q->head = 0;
	q->tail = 0;
	q->count = 0;

	LOG_DBG("PSRAM queue init: addr=0x%06x size=%d item=%d cap=%d",
		base_addr, region_size, item_size, q->capacity);

	return 0;
}

int psram_queue_put(struct psram_queue *q, const void *item)
{
	if (q->count >= q->capacity) {
		return -ENOMEM;
	}

	uint32_t addr = q->base_addr + (q->tail * q->item_size);

	int err = psram_write(addr, item, q->item_size);

	if (err) {
		LOG_ERR("PSRAM queue write failed at 0x%06x, err %d", addr, err);
		return err;
	}

	q->tail = (q->tail + 1) % q->capacity;
	q->count++;

	return 0;
}

int psram_queue_get(struct psram_queue *q, void *item)
{
	if (q->count == 0) {
		return -ENODATA;
	}

	uint32_t addr = q->base_addr + (q->head * q->item_size);

	int err = psram_read(addr, item, q->item_size);

	if (err) {
		LOG_ERR("PSRAM queue read failed at 0x%06x, err %d", addr, err);
		return err;
	}

	q->head = (q->head + 1) % q->capacity;
	q->count--;

	return 0;
}

int psram_queue_peek(struct psram_queue *q, void *item)
{
	if (q->count == 0) {
		return -ENODATA;
	}

	uint32_t addr = q->base_addr + (q->head * q->item_size);

	return psram_read(addr, item, q->item_size);
}

uint16_t psram_queue_count(const struct psram_queue *q)
{
	return q->count;
}

bool psram_queue_is_empty(const struct psram_queue *q)
{
	return (q->count == 0);
}

bool psram_queue_is_full(const struct psram_queue *q)
{
	return (q->count >= q->capacity);
}

void psram_queue_reset(struct psram_queue *q)
{
	q->head = 0;
	q->tail = 0;
	q->count = 0;
}
