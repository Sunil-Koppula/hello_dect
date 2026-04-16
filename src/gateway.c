/*
 * Gateway main loop and pairing handler for DECT NR+ mesh network.
 *
 * Handles the gateway/anchor side of the 4-step pairing:
 *   Sensor → Gateway:  PAIR_REQUEST   → handle_pair_request() sends PAIR_RESPONSE
 *   Sensor → Gateway:  PAIR_CONFIRM   → gateway sends PAIR_ACK, stores sensor
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "gateway.h"
#include "protocol.h"
#include "product_info.h"
#include "mesh.h"
#include "radio.h"
#include "storage.h"
#include "queue.h"
#include "tracker.h"

LOG_MODULE_REGISTER(gateway, CONFIG_MAIN_LOG_LEVEL);

int gateway_init(void)
{
	tracker_init();

	LOG_INF("Gateway init: infra=%d sensors=%d mesh=%d",
		storage_infra_count(), storage_sensor_count(), storage_mesh_count());

	return 0;
}

void gateway_process_rx(const uint8_t *data, uint16_t sender_id, int16_t rssi_2)
{
	switch (data[0]) {
	case PACKET_PAIR_REQUEST:
		handle_pair_request((const pair_request_t *)data, sender_id, rssi_2);
		break;

	case PACKET_PAIR_CONFIRM:
	{
		handle_pair_confirm((const pair_confirm_t *)data, sender_id, rssi_2);
		break;
	}

	default:
		break;
	}
}

void gateway_main(void)
{
	int err;

	gateway_init();

	while (1) {
		/* RX window. */
		err = receive(1, 15);
		if (err) {
			LOG_ERR("Reception failed, err %d", err);
			return;
		}

		k_sem_take(&operation_sem, K_FOREVER);

		/* Process received messages. */
		struct rx_data_item rx_item;
		int rx_count = 0;

		while (rx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
		       rx_queue_get(&rx_item, K_NO_WAIT) == 0) {
			gateway_process_rx(rx_item.data, rx_item.sender_id, rx_item.rssi_2);
			rx_count++;
		}

		/* Transmit queued packets. */
		struct tx_data_item tx_item;
		int tx_count = 0;

		while (tx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
		       tx_queue_get(&tx_item, K_NO_WAIT) == 0) {
			err = transmit(0, tx_item.data, tx_item.data_len, 1);
			if (err) {
				LOG_ERR("TX failed, err %d", err);
				break;
			}
			k_sem_take(&operation_sem, K_FOREVER);
			tx_count++;
		}
	}
}
