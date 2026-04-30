/*
 * Priority message queues for DECT NR+ data (RX and TX)
 *
 * Each direction has small and large variants, each with three k_msgq
 * (high, medium, low priority) in internal RAM.
 * get() drains in order: high → medium → low.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "queue.h"

LOG_MODULE_REGISTER(queue, CONFIG_QUEUE_LOG_LEVEL);

/* RX small queues: high, medium, low priority. */
K_MSGQ_DEFINE(small_rx_msgq_hi,  sizeof(struct rx_small_data_item), QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(small_rx_msgq_med, sizeof(struct rx_small_data_item), QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(small_rx_msgq_lo,  sizeof(struct rx_small_data_item), QUEUE_DEPTH, 4);

/* RX large queues: high, medium, low priority. */
K_MSGQ_DEFINE(large_rx_msgq_hi,  sizeof(struct rx_large_data_item), QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(large_rx_msgq_med, sizeof(struct rx_large_data_item), QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(large_rx_msgq_lo,  sizeof(struct rx_large_data_item), QUEUE_DEPTH, 4);

/* TX small queues: high, medium, low priority. */
K_MSGQ_DEFINE(tx_small_msgq_hi,  sizeof(struct tx_small_data_item), QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(tx_small_msgq_med, sizeof(struct tx_small_data_item), QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(tx_small_msgq_lo,  sizeof(struct tx_small_data_item), QUEUE_DEPTH, 4);

/* TX large queues: high, medium, low priority. */
K_MSGQ_DEFINE(tx_large_msgq_hi,  sizeof(struct tx_large_data_item), QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(tx_large_msgq_med, sizeof(struct tx_large_data_item), QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(tx_large_msgq_lo,  sizeof(struct tx_large_data_item), QUEUE_DEPTH, 4);

static struct k_msgq *rx_small_queues[QUEUE_PRIO_COUNT] = {
	[QUEUE_PRIO_HIGH]   = &small_rx_msgq_hi,
	[QUEUE_PRIO_MEDIUM] = &small_rx_msgq_med,
	[QUEUE_PRIO_LOW]    = &small_rx_msgq_lo,
};

static struct k_msgq *rx_large_queues[QUEUE_PRIO_COUNT] = {
	[QUEUE_PRIO_HIGH]   = &large_rx_msgq_hi,
	[QUEUE_PRIO_MEDIUM] = &large_rx_msgq_med,
	[QUEUE_PRIO_LOW]    = &large_rx_msgq_lo,
};

static struct k_msgq *tx_small_queues[QUEUE_PRIO_COUNT] = {
	[QUEUE_PRIO_HIGH]   = &tx_small_msgq_hi,
	[QUEUE_PRIO_MEDIUM] = &tx_small_msgq_med,
	[QUEUE_PRIO_LOW]    = &tx_small_msgq_lo,
};

static struct k_msgq *tx_large_queues[QUEUE_PRIO_COUNT] = {
	[QUEUE_PRIO_HIGH]   = &tx_large_msgq_hi,
	[QUEUE_PRIO_MEDIUM] = &tx_large_msgq_med,
	[QUEUE_PRIO_LOW]    = &tx_large_msgq_lo,
};

int rx_small_queue_put(const struct rx_small_data_item *item, enum queue_priority prio)
{
	if (prio >= QUEUE_PRIO_COUNT) {
		prio = QUEUE_PRIO_LOW;
	}

	int err = k_msgq_put(rx_small_queues[prio], item, K_NO_WAIT);

	if (err) {
		LOG_WRN("RX small queue (prio %d) full, dropping from device %d",
			prio, item->sender_id);
	}

	return err;
}

int rx_small_queue_get(struct rx_small_data_item *item, k_timeout_t timeout)
{
	/* Drain high → medium → low. */
	if (k_msgq_get(rx_small_queues[QUEUE_PRIO_HIGH], item, K_NO_WAIT) == 0) {
		return 0;
	}

	if (k_msgq_get(rx_small_queues[QUEUE_PRIO_MEDIUM], item, K_NO_WAIT) == 0) {
		return 0;
	}

	return k_msgq_get(rx_small_queues[QUEUE_PRIO_LOW], item, timeout);
}

int rx_large_queue_put(const struct rx_large_data_item *item, enum queue_priority prio)
{
	if (prio >= QUEUE_PRIO_COUNT) {
		prio = QUEUE_PRIO_LOW;
	}

	int err = k_msgq_put(rx_large_queues[prio], item, K_NO_WAIT);

	if (err) {
		LOG_WRN("RX large queue (prio %d) full, dropping from device %d",
			prio, item->sender_id);
	}

	return err;
}

int rx_large_queue_get(struct rx_large_data_item *item, k_timeout_t timeout)
{
	/* Drain high → medium → low. */
	if (k_msgq_get(rx_large_queues[QUEUE_PRIO_HIGH], item, K_NO_WAIT) == 0) {
		return 0;
	}

	if (k_msgq_get(rx_large_queues[QUEUE_PRIO_MEDIUM], item, K_NO_WAIT) == 0) {
		return 0;
	}

	return k_msgq_get(rx_large_queues[QUEUE_PRIO_LOW], item, timeout);
}

int tx_small_queue_put(const void *data, size_t data_len, enum queue_priority prio)
{
	struct tx_small_data_item item = { 0 };

	if (prio >= QUEUE_PRIO_COUNT) {
		prio = QUEUE_PRIO_LOW;
	}

	if (data_len > QUEUE_DATA_MIN) {
		LOG_WRN("TX small data too large (%d > %d), truncating",
			data_len, QUEUE_DATA_MIN);
		data_len = QUEUE_DATA_MIN;
	}

	item.data_len = data_len;
	memcpy(item.data, data, data_len);

	int err = k_msgq_put(tx_small_queues[prio], &item, K_NO_WAIT);

	if (err) {
		LOG_WRN("TX small queue (prio %d) full, dropping packet", prio);
	}

	return err;
}

int tx_small_queue_get(struct tx_small_data_item *item, k_timeout_t timeout)
{
	/* Drain high → medium → low. */
	if (k_msgq_get(tx_small_queues[QUEUE_PRIO_HIGH], item, K_NO_WAIT) == 0) {
		return 0;
	}

	if (k_msgq_get(tx_small_queues[QUEUE_PRIO_MEDIUM], item, K_NO_WAIT) == 0) {
		return 0;
	}

	return k_msgq_get(tx_small_queues[QUEUE_PRIO_LOW], item, timeout);
}

int tx_large_queue_put(const void *data, size_t data_len, enum queue_priority prio)
{
	struct tx_large_data_item item = { 0 };

	if (prio >= QUEUE_PRIO_COUNT) {
		prio = QUEUE_PRIO_LOW;
	}

	if (data_len > QUEUE_DATA_MAX) {
		LOG_WRN("TX large data too large (%d > %d), truncating",
			data_len, QUEUE_DATA_MAX);
		data_len = QUEUE_DATA_MAX;
	}

	item.data_len = data_len;
	memcpy(item.data, data, data_len);

	int err = k_msgq_put(tx_large_queues[prio], &item, K_NO_WAIT);

	if (err) {
		LOG_WRN("TX large queue (prio %d) full, dropping packet", prio);
	}

	return err;
}

int tx_large_queue_get(struct tx_large_data_item *item, k_timeout_t timeout)
{
	/* Drain high → medium → low. */
	if (k_msgq_get(tx_large_queues[QUEUE_PRIO_HIGH], item, K_NO_WAIT) == 0) {
		return 0;
	}

	if (k_msgq_get(tx_large_queues[QUEUE_PRIO_MEDIUM], item, K_NO_WAIT) == 0) {
		return 0;
	}

	return k_msgq_get(tx_large_queues[QUEUE_PRIO_LOW], item, timeout);
}
