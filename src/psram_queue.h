#ifndef PSRAM_QUEUE_H
#define PSRAM_QUEUE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * PSRAM-backed ring buffer queue.
 *
 * Data lives in external PSRAM, metadata in internal RAM.
 * Supports multiple independent queues at different PSRAM base addresses.
 *
 * PSRAM overflow layout (2MB total, ~113KB per queue):
 *   0x000000 — 0x055554  RX overflow high   (~530 items @ 216B)
 *   0x055555 — 0x0AAAA9  RX overflow medium
 *   0x0AAAAA — 0x0FFFFE  RX overflow low
 *   0x100000 — 0x155554  TX overflow high   (~530 items @ 212B)
 *   0x155555 — 0x1AAAA9  TX overflow medium
 *   0x1AAAAA — 0x1FFFFE  TX overflow low
 *   0x200000 — 0x7FFFFF  Free (6MB)
 */

/* Queue instance — metadata in internal RAM. */
struct psram_queue {
	uint32_t base_addr;    /* PSRAM start address for this queue */
	uint32_t region_size;  /* total PSRAM bytes allocated to this queue */
	uint16_t item_size;    /* size of each item in bytes */
	uint16_t capacity;     /* max items (region_size / item_size) */
	uint16_t head;         /* read index */
	uint16_t tail;         /* write index */
	uint16_t count;        /* current number of items */
};

/* Initialize a PSRAM queue instance.
 * base_addr: starting address in PSRAM
 * region_size: bytes allocated in PSRAM for this queue
 * item_size: size of each queue item */
int psram_queue_init(struct psram_queue *q, uint32_t base_addr,
		     uint32_t region_size, uint16_t item_size);

/* Put an item into the queue. Returns 0 on success, -ENOMEM if full. */
int psram_queue_put(struct psram_queue *q, const void *item);

/* Get an item from the queue. Returns 0 on success, -ENODATA if empty. */
int psram_queue_get(struct psram_queue *q, void *item);

/* Peek at the front item without removing. Returns 0 on success. */
int psram_queue_peek(struct psram_queue *q, void *item);

/* Get current item count. */
uint16_t psram_queue_count(const struct psram_queue *q);

/* Check if queue is empty. */
bool psram_queue_is_empty(const struct psram_queue *q);

/* Check if queue is full. */
bool psram_queue_is_full(const struct psram_queue *q);

/* Reset queue (discard all items). */
void psram_queue_reset(struct psram_queue *q);

#endif /* PSRAM_QUEUE_H */
