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

LOG_MODULE_REGISTER(sensor, CONFIG_SENSOR_LOG_LEVEL);

static uint16_t paired_device_id;
static uint8_t  paired_device_type;

static int sensor_init(void)
{
	tracker_init();

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

	tracker_add(0, tid, PACKET_PAIR_REQUEST, PAIR_TIMEOUT_MS, PAIR_MAX_RETRIES);
	send_pair_request(0, tid);

	return 0;
}

static void sensor_process_rx(const uint8_t *data, uint16_t sender_id, int16_t rssi_2)
{
	switch (data[0]) {
	case PACKET_PAIR_RESPONSE: {
		const pair_response_t *resp = (const pair_response_t *)data;

		if (resp->hdr.device_id != radio_get_device_id()) {
			break;
		}

		LOG_INF("PAIR_RESPONSE from %s ID:%d: status 0x%02x, hop %d",
			device_type_str(resp->hdr.device_type), sender_id,
			resp->hdr.status, resp->hop_num);

		/* Find and remove the request tracker by its tracking ID. */
		int idx = tracker_find_by_tracking_id(resp->hdr.tracking_id);
		if (idx >= 0) {
			tracker_remove(idx);
		}

		if (resp->hdr.status != STATUS_SUCCESS) {
			LOG_WRN("PAIR_RESPONSE failed: status 0x%02x", resp->hdr.status);
			break;
		}

		paired_device_id = sender_id;
		paired_device_type = resp->hdr.device_type;

		/* Send confirm with a new tracking ID. */
		uint8_t tid = tracker_next_id();

		tracker_add(sender_id, tid, PACKET_PAIR_CONFIRM, PAIR_TIMEOUT_MS, PAIR_MAX_RETRIES);
		send_pair_confirm(0, sender_id, tid, STATUS_SUCCESS);

		LOG_INF("Sending PAIR_CONFIRM to %s ID:%d (tid: %d)",
			device_type_str(resp->hdr.device_type), sender_id, tid);
		break;
	}

	case PACKET_PAIR_ACK: {
		const pair_ack_t *ack = (const pair_ack_t *)data;

		if (ack->hdr.device_id != radio_get_device_id()) {
			break;
		}

		LOG_INF("PAIR_ACK from %s ID:%d: status 0x%02x",
			device_type_str(ack->hdr.device_type), sender_id, ack->hdr.status);

		/* Find and remove the confirm tracker. */
		int idx = tracker_find_by_tracking_id(ack->hdr.tracking_id);
		if (idx >= 0) {
			tracker_remove(idx);
		}

		if (ack->hdr.status != STATUS_SUCCESS) {
			LOG_WRN("PAIR_ACK failed: status 0x%02x", ack->hdr.status);
			break;
		}

		/* Store to EEPROM partition 1. */
		storage_infra_clear();
		infra_entry_t entry = {
			.device_type = paired_device_type,
			.device_id = paired_device_id,
			.hop_num = 1,
			.rssi_2 = rssi_2,
		};
		int err = storage_infra_add(&entry);

		if (err) {
			LOG_ERR("Failed to store pairing, err %d", err);
		} else {
			LOG_INF("Paired with %s ID:%d and stored",
				device_type_str(paired_device_type), paired_device_id);
		}
		break;
	}

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
			err = sensor_init();
			if (err) {
				LOG_ERR("Sensor init failed, err %d", err);
				state = MAIN_SUB_ERROR;
				break;
			}
			state = MAIN_SUB_RX_WINDOW;
			break;

		case MAIN_SUB_RX_WINDOW:
			err = receive(1, 30);
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
				sensor_process_rx(rx_item.data, rx_item.sender_id, rx_item.rssi_2);
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
