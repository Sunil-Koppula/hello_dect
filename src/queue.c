/*
 * Priority message queues for DECT NR+ data (RX and TX)
 *
 * Each direction has two k_msgq (high and low priority).
 * get() always drains the high-priority queue first.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "queue.h"

LOG_MODULE_REGISTER(queue, CONFIG_RADIO_LOG_LEVEL);

/* RX queues: high and low priority. */
K_MSGQ_DEFINE(rx_msgq_hi, sizeof(struct rx_data_item), RX_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(rx_msgq_lo, sizeof(struct rx_data_item), RX_QUEUE_DEPTH, 4);

/* TX queues: high and low priority. */
K_MSGQ_DEFINE(tx_msgq_hi, sizeof(struct tx_data_item), TX_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(tx_msgq_lo, sizeof(struct tx_data_item), TX_QUEUE_DEPTH, 4);

static struct k_msgq *rx_queues[QUEUE_PRIO_COUNT] = {
	[QUEUE_PRIO_HIGH] = &rx_msgq_hi,
	[QUEUE_PRIO_LOW]  = &rx_msgq_lo,
};

static struct k_msgq *tx_queues[QUEUE_PRIO_COUNT] = {
	[QUEUE_PRIO_HIGH] = &tx_msgq_hi,
	[QUEUE_PRIO_LOW]  = &tx_msgq_lo,
};

int rx_queue_put(const struct rx_data_item *item, enum queue_priority prio)
{
	if (prio >= QUEUE_PRIO_COUNT) {
		prio = QUEUE_PRIO_LOW;
	}

	int err = k_msgq_put(rx_queues[prio], item, K_NO_WAIT);

	if (err) {
		LOG_WRN("RX queue (prio %d) full, dropping message from device %d",
			prio, item->sender_id);
	}

	return err;
}

int rx_queue_get(struct rx_data_item *item, k_timeout_t timeout)
{
	/* Drain high priority first. */
	int err = k_msgq_get(&rx_msgq_hi, item, K_NO_WAIT);

	if (err == 0) {
		return 0;
	}

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

	int err = k_msgq_put(tx_queues[prio], &item, K_NO_WAIT);

	if (err) {
		LOG_WRN("TX queue (prio %d) full, dropping packet", prio);
	}

	return err;
}

int tx_queue_get(struct tx_data_item *item, k_timeout_t timeout)
{
	/* Drain high priority first. */
	int err = k_msgq_get(&tx_msgq_hi, item, K_NO_WAIT);

	if (err == 0) {
		return 0;
	}

	return k_msgq_get(&tx_msgq_lo, item, timeout);
}
