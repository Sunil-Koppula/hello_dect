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
#include "anchor.h"
#include "protocol.h"
#include "product_info.h"
#include "mesh.h"
#include "radio.h"
#include "storage.h"
#include "queue.h"
#include "tracker.h"

LOG_MODULE_REGISTER(anchor, CONFIG_MAIN_LOG_LEVEL);

#define PAIR_TIMEOUT_MS     500
#define PAIR_MAX_RETRIES    5

static uint16_t paired_device_id;
static uint8_t  paired_device_type;

int anchor_init(void)
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

void anchor_process_rx(const uint8_t *data, uint16_t sender_id, int16_t rssi_2)
{
	switch (data[0]) {
	case PACKET_PAIR_RESPONSE: {
		const pair_response_t *resp = (const pair_response_t *)data;

		if (resp->device_id != radio_get_device_id()) {
			break;
		}

		LOG_INF("PAIR_RESPONSE from %s ID:%d: status 0x%02x, hop %d",
			device_type_str(resp->device_type), sender_id,
			resp->status, resp->hop_num);

		/* Find and remove the request tracker by tracking ID from the packet. */
		int idx = tracker_find_by_tracking_id(resp->tracking_id);
		if (idx >= 0) {
			tracker_remove(idx);
		}

		if (resp->status == STATUS_SUCCESS) {
			/* Check if already stored. */
			infra_entry_t entry;

			for (int i = 0; i < storage_infra_count(); i++) {
				if (storage_infra_get(i, &entry) == 0 &&
				    entry.device_id == sender_id) {
					LOG_INF("Device %d already stored in infra, skipping",
						sender_id);
					return;
				}
			}

			if (storage_infra_count() >= STORAGE_PART1_MAX_ENTRIES) {
				LOG_WRN("Infra storage full, cannot add device %d", sender_id);
				return;
			}

			/* Send confirm with a new tracking ID. */
			uint8_t tid = tracker_next_id();

			LOG_INF("Sending PAIR_CONFIRM to %s ID:%d (tid: %d)",
				device_type_str(resp->device_type), sender_id, tid);
			tracker_add(sender_id, tid, PACKET_PAIR_CONFIRM, PAIR_TIMEOUT_MS, PAIR_MAX_RETRIES);
			send_pair_confirm(0, sender_id, tid, STATUS_SUCCESS);
		} else {
			LOG_WRN("PAIR_RESPONSE failed: status 0x%02x", resp->status);
		}
		break;
	}

	case PACKET_PAIR_ACK: {
		const pair_ack_t *ack = (const pair_ack_t *)data;

		if (ack->device_id != radio_get_device_id()) {
			break;
		}

		LOG_INF("PAIR_ACK from %s ID:%d: status 0x%02x",
			device_type_str(ack->device_type), sender_id, ack->status);

		/* Find and remove the confirm tracker. */
		int idx = tracker_find_by_tracking_id(ack->tracking_id);
		if (idx >= 0) {
			tracker_remove(idx);
		}

		if (ack->status == STATUS_SUCCESS) {
			/* Check if already stored. */
			infra_entry_t entry;

			for (int i = 0; i < storage_infra_count(); i++) {
				if (storage_infra_get(i, &entry) == 0 &&
				    entry.device_id == sender_id) {
					LOG_INF("Device %d already stored in infra, skipping",
						sender_id);
					return;
				}
			}

			if (storage_infra_count() >= STORAGE_PART1_MAX_ENTRIES) {
				LOG_WRN("Infra storage full, cannot add device %d", sender_id);
				return;
			}

			entry.device_id = sender_id;
			entry.device_type = ack->device_type;
			entry.hop_num = 0;
			entry.rssi_2 = rssi_2;
			int err = storage_infra_add(&entry);

			if (err) {
				LOG_ERR("Failed to store paired device, err %d", err);
				return;
			}

			LOG_INF("Device %d paired and stored in infra (total %d)",
				sender_id, storage_infra_count());
		} else {
			LOG_WRN("PAIR_ACK failed: status 0x%02x", ack->status);
		}
		break;
	}

	default:
		break;
	}
}

void anchor_main(void)
{
	int err;

	anchor_init();

	while (1) {
		err = receive(1, 15);
		if (err) {
			LOG_ERR("Reception failed, err %d", err);
			return;
		}

		k_sem_take(&operation_sem, K_FOREVER);

		struct rx_data_item rx_item;
		int rx_count = 0;

		while (rx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
		       rx_queue_get(&rx_item, K_NO_WAIT) == 0) {
			anchor_process_rx(rx_item.data, rx_item.sender_id, rx_item.rssi_2);
			rx_count++;
		}

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

		tracker_tick(tracker_default_expired_cb);
	}
}
