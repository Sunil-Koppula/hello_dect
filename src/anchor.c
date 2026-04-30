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
#include "data.h"

LOG_MODULE_REGISTER(anchor, CONFIG_ANCHOR_LOG_LEVEL);

static int anchor_init(void)
{
	int infra = storage_infra_count();

	LOG_INF("Anchor init: infra=%d sensors=%d", infra, storage_sensor_count());

	/* If we already have upstream infra connections, we're paired. */
	if (infra > 0) {
		infra_entry_t entry;
		LOG_INF("Already paired with: ");

		for (int i = 0; i < infra; i++) {
			int err = storage_infra_get(i, &entry);
			if (err) {
				LOG_ERR("Failed to read infra entry %d, err %d", i, err);
				continue;
			}
			LOG_INF("Infra entry %d: %s ID:%d (hop:%d)", i,
				device_type_str(entry.device_type),
				entry.device_id, entry.hop_num);
		}

		product_info_update_hop();

		if (infra >= MAX_ANCHORS) {
			return 0;
		}
	}
	storage_infra_clear();
	storage_sensor_clear();

	LOG_INF("Sending PAIR REQUEST!");

	tracker_init();
	data_init();
	uint8_t tid = tracker_next_id();

	tracker_add(radio_get_device_id(), 0, tid, PACKET_PAIR_REQUEST, 5 * PAIR_TIMEOUT_MS, PAIR_MAX_RETRIES, NULL, 0);
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

	case PACKET_JOINED_NETWORK:
		handle_joined_network((const joined_network_t *)data, sender_id, rssi_2);
		break;
	
	case PACKET_JOINED_NETWORK_ACK:
		handle_joined_network_ack((const joined_network_ack_t *)data, sender_id, rssi_2);
		break;
	
	case PACKET_PING_DEVICE:
		handle_ping_device((const ping_device_t *)data, sender_id, rssi_2);
		break;

	case PACKET_PING_ACK:
		handle_ping_ack((const ping_ack_t *)data, sender_id, rssi_2);
		break;
	
	case PACKET_DEVICE_UPDATED:
		handle_device_updated((const device_updated_t *)data, sender_id, rssi_2);
		break;

	case PACKET_DEVICE_UPDATED_ACK:
		handle_device_updated_ack((const device_updated_ack_t *)data, sender_id, rssi_2);
		break;

	case PACKET_REPAIR_REQUEST:
		handle_repair_request((const repair_request_t *)data, sender_id, rssi_2);
		break;

	case PACKET_REPAIR_RESPONSE:
		handle_repair_response((const repair_response_t *)data, sender_id, rssi_2);
		break;

	case PACKET_SYNC_TIME:
		handle_sync_time((const sync_time_t *)data, sender_id, rssi_2);
		break;

	case PACKET_SYNC_TIME_ACK:
		handle_sync_time_ack((const sync_time_ack_t *)data, sender_id, rssi_2);
		break;

	case PACKET_DATA_INIT:
		handle_data_init((const data_init_t *)data, sender_id, rssi_2);
		break;

	case PACKET_DATA_INIT_ACK:
		handle_data_init_ack((const data_init_ack_t *)data, sender_id, rssi_2);
		break;

	case PACKET_DATA_CHUNK:
		handle_data_chunk((const data_chunk_t *)data, sender_id, rssi_2);
		break;

	case PACKET_DATA_CHUNK_ACK:
		handle_data_chunk_ack((const data_chunk_ack_t *)data, sender_id, rssi_2);
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
			err = receive(1, 25);
			if (err) {
				LOG_ERR("Reception failed, err %d", err);
				state = MAIN_SUB_ERROR;
				break;
			}
			k_sem_take(&operation_sem, K_FOREVER);
			state = MAIN_SUB_RX_PROCESS;
			break;

		case MAIN_SUB_RX_PROCESS: {
			struct rx_small_data_item rx_small_item;
			struct rx_large_data_item rx_large_item;
			int rx_count = 0;

			while (rx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
			       rx_small_queue_get(&rx_small_item, K_NO_WAIT) == 0) {
				anchor_process_rx(rx_small_item.data, rx_small_item.sender_id, rx_small_item.rssi_2);
				rx_count++;
			}

			while (rx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
			       rx_large_queue_get(&rx_large_item, K_NO_WAIT) == 0) {
				anchor_process_rx(rx_large_item.data, rx_large_item.sender_id, rx_large_item.rssi_2);
				rx_count++;
			}
			state = MAIN_SUB_TX_PROCESS;
			break;
		}

		case MAIN_SUB_TX_PROCESS: {
			struct tx_small_data_item tx_small_item;
			struct tx_large_data_item tx_large_item;
			int tx_count = 0;

			while (tx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
			       tx_small_queue_get(&tx_small_item, K_NO_WAIT) == 0) {
				err = transmit(0, tx_small_item.data, tx_small_item.data_len, 0);
				if (err) {
					LOG_ERR("TX failed, err %d", err);
					break;
				}
				k_sem_take(&operation_sem, K_FOREVER);
				tx_count++;
			}

			while (tx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
			       tx_large_queue_get(&tx_large_item, K_NO_WAIT) == 0) {
				err = transmit(0, tx_large_item.data, tx_large_item.data_len, 0);
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
			mesh_tick();
			tracker_tick(tracker_default_expired_cb);
			data_tick();
			mesh_time_check_milestone();
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
