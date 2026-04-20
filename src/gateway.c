/*
 * Gateway main loop and pairing handler for DECT NR+ mesh network.
 *
 * Handles the gateway/anchor side of the 4-step pairing:
 *   Sensor → Gateway:  PAIR_REQUEST   → handle_pair_request() sends PAIR_RESPONSE
 *   Sensor → Gateway:  PAIR_CONFIRM   → gateway sends PAIR_ACK, stores sensor
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include "main_sub.h"
#include "protocol.h"
#include "product_info.h"
#include "mesh.h"
#include "radio.h"
#include "storage.h"
#include "queue.h"
#include "tracker.h"

LOG_MODULE_REGISTER(gateway, CONFIG_GATEWAY_LOG_LEVEL);

static int gateway_init(void)
{
	tracker_init();

	LOG_INF("Gateway init: infra=%d sensors=%d mesh=%d",
		storage_infra_count(), storage_sensor_count(), storage_mesh_count());

	if (storage_infra_count() > 0) {
		LOG_INF("Already paired with: ");

		for (int i = 0; i < storage_infra_count(); i++) {
			infra_entry_t entry;
			int err = storage_infra_get(i, &entry);
			if (err) {
				LOG_ERR("Failed to read infra entry %d, err %d", i, err);
				continue;
			}
			LOG_INF("Infra entry %d: %s ID:%d (hop:%d)", i,
				device_type_str(entry.device_type),
				entry.device_id, entry.hop_num);
		}
	}
	return 0;
}

static void gateway_process_rx(const uint8_t *data, uint16_t sender_id, int16_t rssi_2)
{
	switch (data[0]) {
		case PACKET_PAIR_REQUEST:
			handle_pair_request((const pair_request_t *)data, sender_id, rssi_2);
			break;

		case PACKET_PAIR_RESPONSE:
			/* Gateway should never receive PAIR_RESPONSE, ignore. */
			break;

		case PACKET_PAIR_CONFIRM:
			handle_pair_confirm((const pair_confirm_t *)data, sender_id, rssi_2);
			break;

		case PACKET_PAIR_ACK:
			/* Gateway should never receive PAIR_ACK, ignore. */
			break;

		case PACKET_JOINED_NETWORK:
			handle_joined_network((const joined_network_t *)data, sender_id, rssi_2);
			break;

		case PACKET_JOINED_NETWORK_ACK:
			/* Gateway should never receive JOINED_NETWORK_ACK, ignore. */
			break;

		default:
			break;
	}
}

void gateway_main(void)
{
	int err;
	main_sub_state_t state = MAIN_SUB_INIT;

	while (1) {
		switch (state) {
		case MAIN_SUB_INIT:
			err = gateway_init();
			if (err) {
				LOG_ERR("Gateway init failed, err %d", err);
				state = MAIN_SUB_ERROR;
				break;
			}
			state = MAIN_SUB_RX_WINDOW;
			break;

		case MAIN_SUB_RX_WINDOW:
			err = receive(1, 15);
			if (err) {
				LOG_ERR("Reception failed, err %d", err);
				state = MAIN_SUB_ERROR;
				break;
			}
			k_sem_take(&operation_sem, K_FOREVER);
			state = MAIN_SUB_RX_PROCESS;
			break;

		case MAIN_SUB_RX_PROCESS: {
			struct rx_data_item rx_item;
			int rx_count = 0;

			while (rx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
			       rx_queue_get(&rx_item, K_NO_WAIT) == 0) {
				gateway_process_rx(rx_item.data, rx_item.sender_id, rx_item.rssi_2);
				rx_count++;
			}
			state = MAIN_SUB_TX_PROCESS;
			break;
		}

		case MAIN_SUB_TX_PROCESS: {
			struct tx_data_item tx_item;
			int tx_count = 0;

			while (tx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
			       tx_queue_get(&tx_item, K_NO_WAIT) == 0) {
				err = transmit(0, tx_item.data, tx_item.data_len, 0);
				if (err) {
					LOG_ERR("TX failed, err %d", err);
					break;
				}
				k_sem_take(&operation_sem, K_FOREVER);
				tx_count++;
			}
			state = MAIN_SUB_TRACKER;
			break;
		}

		case MAIN_SUB_TRACKER:
			tracker_tick(tracker_default_expired_cb);
			state = MAIN_SUB_RX_WINDOW;
			break;

		case MAIN_SUB_ERROR:
			LOG_ERR("Gateway in error state, waiting 10s before retry");
			k_msleep(10000);
			state = MAIN_SUB_INIT;
			break;

		default:
			LOG_ERR("Gateway in unknown state %d", state);
			// Reset the device to recover from unknown state
			sys_reboot(SYS_REBOOT_COLD);
			break;
		}
	}
}
