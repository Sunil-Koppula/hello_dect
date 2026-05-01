/*
 * Sensor pairing state machine for DECT NR+ mesh network.
 *
 * 4-step pairing flow:
 *   1. Sensor broadcasts PAIR_REQUEST
 *   2. Waits for PAIR_RESPONSE (with tracking + timeout)
 *   3. Sends PAIR_CONFIRM to responder
 *   4. Waits for PAIR_ACK, stores pairing to EEPROM
 *
 * Tracking IDs are looked up from the tracker pool — no static TID variables.
 * A sensor can only pair with one gateway or anchor.
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

LOG_MODULE_REGISTER(sensor, CONFIG_SENSOR_LOG_LEVEL);

static uint16_t paired_device_id;
static uint8_t  paired_device_type;

static int sensor_init(void)
{
	tracker_init();
	data_init();

	storage_infra_clear();

	/* Check EEPROM partition 1 — sensor can pair with only one device. */
	if (storage_infra_count() > 0) {
		infra_entry_t entry;
		int err = storage_infra_get(0, &entry);

		if (err == 0) {
			paired_device_id = entry.device_id;
			paired_device_type = entry.device_type;
			LOG_INF("Already paired with %s ID:%d (hop:%d)",
				device_type_str(entry.device_type),
				entry.device_id, entry.hop_num);
			return 0;
		}
	}

	/* Not paired — start pairing. */
	LOG_INF("Sensor not paired, sending PAIR_REQUEST");

	uint8_t tid = tracker_next_id();

	tracker_add(radio_get_device_id(), 0, tid, PACKET_PAIR_REQUEST, 5 * PAIR_TIMEOUT_MS, PAIR_MAX_RETRIES, NULL, 0);
	send_pair_request(0, tid);

	return 0;
}

static void sensor_process_rx(const uint8_t *data, uint16_t sender_id, int16_t rssi_2)
{
	switch (data[0]) {
		case PACKET_PAIR_REQUEST:
			/* Sensor should never receive PAIR_REQUEST, ignore. */
			break;

		case PACKET_PAIR_RESPONSE:
			handle_pair_response((const pair_response_t *)data, sender_id, rssi_2);
			break;

		case PACKET_PAIR_CONFIRM:
			/* Sensor should never receive PAIR_CONFIRM, ignore. */
			break;

		case PACKET_PAIR_ACK:
			handle_pair_ack((const pair_ack_t *)data, sender_id, rssi_2);
			break;

		case PACKET_JOINED_NETWORK:
			/* Sensor should never receive JOINED_NETWORK, ignore. */
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
			/* Sensor should never receive DEVICE_UPDATED, ignore. */
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
}

void sensor_main(void)
{
	int err;
	main_sub_state_t state = MAIN_SUB_INIT;

	while (1) {
		switch (state) {
		case MAIN_SUB_INIT:
		{
			err = sensor_init();
			if (err) {
				LOG_ERR("Sensor init failed, err %d", err);
				state = MAIN_SUB_ERROR;
				break;
			}
			state = MAIN_SUB_RX_WINDOW;
			break;
		}

		case MAIN_SUB_RX_WINDOW:
		{
			err = receive(1, 30);
			if (err) {
				LOG_ERR("Reception failed, err %d", err);
				state = MAIN_SUB_ERROR;
				break;
			}
			k_sem_take(&operation_sem, K_FOREVER);
			state = MAIN_SUB_RX_PROCESS;
			break;
		}


		case MAIN_SUB_RX_PROCESS:
		{
			struct rx_data_item rx_item;
			int rx_count = 0;

			while (rx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
			       rx_queue_get(&rx_item, K_NO_WAIT) == 0) {
				sensor_process_rx(rx_item.data, rx_item.sender_id, rx_item.rssi_2);
				rx_count++;
			}

			state = MAIN_SUB_TX_PROCESS;
			break;
		}

		case MAIN_SUB_TX_PROCESS:
		{
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
			mesh_tick();
			tracker_tick(tracker_default_expired_cb);
			data_tick();
			mesh_time_check_milestone();
			state = MAIN_SUB_RX_WINDOW;
			break;

		case MAIN_SUB_ERROR:
			LOG_ERR("Sensor in error state, waiting 10s before retry");
			k_msleep(10000);
			state = MAIN_SUB_INIT;
			break;

		default:
			LOG_ERR("Sensor in unknown state %d", state);
			sys_reboot(SYS_REBOOT_COLD);
			break;
		}
	}
}
