/*
 * Priority message queues for DECT NR+ data (RX and TX)
 *
 * Each direction has three k_msgq (high, medium, low priority) in internal RAM.
 * When internal queue is full, overflow spills to PSRAM (2MB total).
 * When internal queue is drained, refills from PSRAM overflow.
 *
 * get() drains in order: high → medium → low.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "queue.h"
#include "psram_queue.h"

LOG_MODULE_REGISTER(queue, CONFIG_QUEUE_LOG_LEVEL);

/* Internal RAM queues (fast, depth 32 each). */
K_MSGQ_DEFINE(rx_msgq_hi,  sizeof(struct rx_data_item), QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(rx_msgq_med, sizeof(struct rx_data_item), QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(rx_msgq_lo,  sizeof(struct rx_data_item), QUEUE_DEPTH, 4);

K_MSGQ_DEFINE(tx_msgq_hi,  sizeof(struct tx_data_item), QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(tx_msgq_med, sizeof(struct tx_data_item), QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(tx_msgq_lo,  sizeof(struct tx_data_item), QUEUE_DEPTH, 4);

static struct k_msgq *rx_queues[QUEUE_PRIO_COUNT] = {
	[QUEUE_PRIO_HIGH]   = &rx_msgq_hi,
	[QUEUE_PRIO_MEDIUM] = &rx_msgq_med,
	[QUEUE_PRIO_LOW]    = &rx_msgq_lo,
};

static struct k_msgq *tx_queues[QUEUE_PRIO_COUNT] = {
	[QUEUE_PRIO_HIGH]   = &tx_msgq_hi,
	[QUEUE_PRIO_MEDIUM] = &tx_msgq_med,
	[QUEUE_PRIO_LOW]    = &tx_msgq_lo,
};

/*
 * PSRAM overflow queues (slow, large capacity).
 *
 * Layout in PSRAM (2MB total starting at 0x000000):
 *   0x000000 — 0x055554  RX overflow (high/med/low, ~113KB each)
 *   0x100000 — 0x155554  TX overflow (high/med/low, ~113KB each)
 */
#define PSRAM_OVERFLOW_BASE    0x000000
#define PSRAM_OVERFLOW_SIZE    0x200000  /* 2MB total */
#define PSRAM_PER_QUEUE        (PSRAM_OVERFLOW_SIZE / (QUEUE_PRIO_COUNT * 2))

static struct psram_queue rx_overflow[QUEUE_PRIO_COUNT];
static struct psram_queue tx_overflow[QUEUE_PRIO_COUNT];
static bool overflow_ready;

int queue_init(void)
{
	int err;
	uint32_t addr = PSRAM_OVERFLOW_BASE;

	/* RX overflow queues. */
	for (int i = 0; i < QUEUE_PRIO_COUNT; i++) {
		err = psram_queue_init(&rx_overflow[i], addr,
				       PSRAM_PER_QUEUE, sizeof(struct rx_data_item));
		if (err) {
			LOG_ERR("RX overflow init failed (prio %d), err %d", i, err);
			return err;
		}
		addr += PSRAM_PER_QUEUE;
	}

	/* TX overflow queues. */
	for (int i = 0; i < QUEUE_PRIO_COUNT; i++) {
		err = psram_queue_init(&tx_overflow[i], addr,
				       PSRAM_PER_QUEUE, sizeof(struct tx_data_item));
		if (err) {
			LOG_ERR("TX overflow init failed (prio %d), err %d", i, err);
			return err;
		}
		addr += PSRAM_PER_QUEUE;
	}

	overflow_ready = true;

	LOG_INF("Queue overflow init: %d bytes/queue, %d RX cap, %d TX cap",
		PSRAM_PER_QUEUE,
		rx_overflow[0].capacity,
		tx_overflow[0].capacity);

	return 0;
}

/* Try to refill internal queue from PSRAM overflow. */
static void rx_refill_from_overflow(int prio)
{
	if (!overflow_ready || psram_queue_is_empty(&rx_overflow[prio])) {
		return;
	}

	struct rx_data_item item;

	while (psram_queue_get(&rx_overflow[prio], &item) == 0) {
		if (k_msgq_put(rx_queues[prio], &item, K_NO_WAIT) != 0) {
			/* Internal queue full again — put it back. */
			psram_queue_put(&rx_overflow[prio], &item);
			break;
		}
	}
}

static void tx_refill_from_overflow(int prio)
{
	if (!overflow_ready || psram_queue_is_empty(&tx_overflow[prio])) {
		return;
	}

	struct tx_data_item item;

	while (psram_queue_get(&tx_overflow[prio], &item) == 0) {
		if (k_msgq_put(tx_queues[prio], &item, K_NO_WAIT) != 0) {
			psram_queue_put(&tx_overflow[prio], &item);
			break;
		}
	}
}

int rx_queue_put(const struct rx_data_item *item, enum queue_priority prio)
{
	if (prio >= QUEUE_PRIO_COUNT) {
		prio = QUEUE_PRIO_LOW;
	}

	/* Try internal RAM first. */
	int err = k_msgq_put(rx_queues[prio], item, K_NO_WAIT);

	if (err == 0) {
		return 0;
	}

	/* Internal full — spill to PSRAM overflow. */
	if (overflow_ready) {
		err = psram_queue_put(&rx_overflow[prio], item);
		if (err == 0) {
			LOG_DBG("RX prio %d overflow to PSRAM (%d buffered)",
				prio, psram_queue_count(&rx_overflow[prio]));
			return 0;
		}
	}

	LOG_WRN("RX queue (prio %d) full (RAM + PSRAM), dropping from device %d",
		prio, item->sender_id);
	return -ENOMEM;
}

int rx_queue_get(struct rx_data_item *item, k_timeout_t timeout)
{
	/* Drain high → medium → low. */
	for (int p = QUEUE_PRIO_HIGH; p < QUEUE_PRIO_COUNT; p++) {
		if (k_msgq_get(rx_queues[p], item, K_NO_WAIT) == 0) {
			/* Refill from PSRAM if there's overflow data. */
			rx_refill_from_overflow(p);
			return 0;
		}

		/* Internal empty — try PSRAM overflow directly. */
		if (overflow_ready && psram_queue_get(&rx_overflow[p], item) == 0) {
			return 0;
		}
	}

	/* All empty — block on low priority with caller's timeout. */
	return k_msgq_get(&rx_msgq_lo, item, timeout);
}

int tx_queue_put(const void *data, size_t data_len, enum queue_priority prio)
{
	struct tx_data_item item = { 0 };

	if (prio >= QUEUE_PRIO_COUNT) {
		prio = QUEUE_PRIO_LOW;
	}

	if (data_len > QUEUE_DATA_MAX) {
		LOG_WRN("TX data too large (%d > %d), truncating", data_len, QUEUE_DATA_MAX);
		data_len = QUEUE_DATA_MAX;
	}

	item.data_len = data_len;
	memcpy(item.data, data, data_len);

	/* Try internal RAM first. */
	int err = k_msgq_put(tx_queues[prio], &item, K_NO_WAIT);

	if (err == 0) {
		return 0;
	}

	/* Internal full — spill to PSRAM overflow. */
	if (overflow_ready) {
		err = psram_queue_put(&tx_overflow[prio], &item);
		if (err == 0) {
			LOG_DBG("TX prio %d overflow to PSRAM (%d buffered)",
				prio, psram_queue_count(&tx_overflow[prio]));
			return 0;
		}
	}

	LOG_WRN("TX queue (prio %d) full (RAM + PSRAM), dropping packet", prio);
	return -ENOMEM;
}

int tx_queue_get(struct tx_data_item *item, k_timeout_t timeout)
{
	/* Drain high → medium → low. */
	for (int p = QUEUE_PRIO_HIGH; p < QUEUE_PRIO_COUNT; p++) {
		if (k_msgq_get(tx_queues[p], item, K_NO_WAIT) == 0) {
			tx_refill_from_overflow(p);
			return 0;
		}

		if (overflow_ready && psram_queue_get(&tx_overflow[p], item) == 0) {
			return 0;
		}
	}

	return k_msgq_get(&tx_msgq_lo, item, timeout);
}
