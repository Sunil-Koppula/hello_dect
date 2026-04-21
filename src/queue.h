#ifndef QUEUE_H
#define QUEUE_H

#include <stdint.h>
#include <stddef.h>
#include <zephyr/kernel.h>
#include "product_info.h"

#define QUEUE_DATA_MAX 210  /* 15 subslots × 14 bytes/subslot at MCS 2 */
#define QUEUE_DEPTH 32
#define MAX_QUEUE_PROCESS_PER_CYCLE 4

/* Priority levels (drained in order: HIGH → MEDIUM → LOW). */
enum queue_priority {
	QUEUE_PRIO_HIGH   = 0,
	QUEUE_PRIO_MEDIUM = 1,
	QUEUE_PRIO_LOW    = 2,
	QUEUE_PRIO_COUNT,
};

/* Received data item. */
struct rx_data_item {
	uint16_t sender_id;
	int16_t rssi_2;
	uint16_t data_len;
	uint8_t data[QUEUE_DATA_MAX];
};

/* Transmit data item. */
struct tx_data_item {
	uint16_t data_len;
	uint8_t data[QUEUE_DATA_MAX];
};

/* RX queue operations. */
int rx_queue_put(const struct rx_data_item *item, enum queue_priority prio);
int rx_queue_get(struct rx_data_item *item, k_timeout_t timeout);

/* TX queue operations. */
int tx_queue_put(const void *data, size_t data_len, enum queue_priority prio);
int tx_queue_get(struct tx_data_item *item, k_timeout_t timeout);

#endif /* QUEUE_H */
