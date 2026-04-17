/*
 * Anchor pairing and main loop for DECT NR+ mesh network.
 *
 * The anchor is a hybrid device:
 *   - Upstream: pairs with gateway/other anchors (like a sensor, 4-step)
 *   - Downstream: accepts sensor pair requests (like a gateway)
 *
 * Tracking IDs are looked up from the tracker pool — no static TID variables.
 * Partition 1 stores upstream infra devices (max 8).
 * Partition 2 stores connected sensors.
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

LOG_MODULE_REGISTER(anchor, CONFIG_ANCHOR_LOG_LEVEL);

static uint16_t paired_device_id;
static uint8_t  paired_device_type;

static int anchor_init(void)
{
	int infra = storage_infra_count();

	LOG_INF("Anchor init: infra=%d sensors=%d", infra, storage_sensor_count());

	/* If we already have upstream infra connections, we're paired. */
	if (infra > 0) {
		infra_entry_t entry;

		if (storage_infra_get(0, &entry) == 0) {
			paired_device_id = entry.device_id;
			paired_device_type = entry.device_type;
			LOG_INF("Already paired with %s ID:%d (hop:%d)",
				device_type_str(entry.device_type),
				entry.device_id, entry.hop_num);
			return 0;
		}
	}

	LOG_INF("Anchor not paired, Sending PAIR REQUEST!");

	tracker_init();
	uint8_t tid = tracker_next_id();

	tracker_add(0, tid, PACKET_PAIR_REQUEST, PAIR_TIMEOUT_MS, PAIR_MAX_RETRIES);
	send_pair_request(0, tid);

	return 0;
}

static void anchor_process_rx(const uint8_t *data, uint16_t sender_id, int16_t rssi_2)
{
	switch (data[0]) {
	case PACKET_PAIR_REQUEST:
		handle_pair_request((const pair_request_t *)data, sender_id, rssi_2);
		break;
	
	case PACKET_PAIR_RESPONSE:
		handle_pair_response((const pair_response_t *)data, sender_id, rssi_2);
		break;

	case PACKET_PAIR_CONFIRM:
		handle_pair_confirm((const pair_confirm_t *)data, sender_id, rssi_2);
		break;
	
	case PACKET_PAIR_ACK:
		handle_pair_ack((const pair_ack_t *)data, sender_id, rssi_2);
		break;
	
	default:
		break;
	}

	return;
}

void anchor_main(void)
{
	int err;
	main_sub_state_t state = MAIN_SUB_INIT;

	while (1) {
		switch (state) {
		case MAIN_SUB_INIT:
			err = anchor_init();
			if (err) {
				LOG_ERR("Anchor init failed, err %d", err);
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
				anchor_process_rx(rx_item.data, rx_item.sender_id, rx_item.rssi_2);
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
			LOG_ERR("Anchor in error state, waiting 10s before retry");
			k_msleep(10000);
			state = MAIN_SUB_INIT;
			break;

		default:
			LOG_ERR("Anchor in unknown state %d", state);
			sys_reboot(SYS_REBOOT_COLD);
			break;
		}
	}
}
