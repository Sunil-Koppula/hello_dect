/*
 * Anchor pairing and main loop for DECT NR+ mesh network.
 *
 * The anchor is a hybrid device:
 *   - Upstream: pairs with gateway/other anchors (like a sensor, 4-step)
 *   - Downstream: accepts sensor pair requests (like a gateway)
 *
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

LOG_MODULE_REGISTER(anchor, CONFIG_MAIN_LOG_LEVEL);

#define PAIR_RETRY_MAX       5
#define PAIR_RETRY_INTERVAL  10   /* main loop cycles between retries */

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
			LOG_INF("Already paired with %s ID:%d (hop:%d)", device_type_str(entry.device_type), entry.device_id, entry.hop_num);
			return 0;
		}
	}

	LOG_INF("Anchor not paired, starting upstream pairing");

	send_pair_request(0);

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

        LOG_INF("Pair Response from device %s ID:%d: status 0x%02x, hop %d", device_type_str(resp->device_type), sender_id, resp->status, resp->hop_num);

        if (resp->status == STATUS_SUCCESS) {
			switch (resp->device_type) {
				case DEVICE_TYPE_GATEWAY:
					LOG_INF("Paired with gateway device %d", sender_id);
					break;
			}
            paired_device_id = sender_id;
            paired_device_type = resp->device_type;
            LOG_INF("Paired with %s ID:%d", device_type_str(resp->device_type), sender_id);
        } else {
            LOG_WRN("Pair Response failed, retrying...");
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
		/* RX window. */
		err = receive(1, 30);
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
			anchor_process_rx(rx_item.data, rx_item.sender_id, rx_item.rssi_2);
			rx_count++;
		}

		/* Transmit queued packets. */
		struct tx_data_item tx_item;
		int tx_count = 0;

		while (tx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
		       tx_queue_get(&tx_item, K_NO_WAIT) == 0) {
			err = transmit(0, tx_item.data, tx_item.data_len);
			if (err) {
				LOG_ERR("TX failed, err %d", err);
				break;
			}
			k_sem_take(&operation_sem, K_FOREVER);
			tx_count++;
		}

	}
}
