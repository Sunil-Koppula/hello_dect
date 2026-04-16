/*
 * Sensor pairing state machine for DECT NR+ mesh network.
 *
 * 4-step pairing flow with best-RSSI selection:
 *   1. Sensor broadcasts PAIR_REQUEST
 *   2. Collects PAIR_RESPONSE from all responders for ~1 second
 *   3. Picks the responder with best RSSI, sends PAIR_CONFIRM
 *   4. Waits for PAIR_ACK, stores pairing to EEPROM
 *
 * A sensor can only pair with one gateway or anchor.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "sensor.h"
#include "protocol.h"
#include "product_info.h"
#include "mesh.h"
#include "radio.h"
#include "storage.h"
#include "queue.h"

LOG_MODULE_REGISTER(sensor, CONFIG_MAIN_LOG_LEVEL);

static uint16_t paired_device_id;
static uint8_t  paired_device_type;

int sensor_init(void)
{
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
	LOG_INF("Sensor not paired, starting pairing");

	send_pair_request(0, 0);

	return 0;
}

void sensor_process_rx(const uint8_t *data, uint16_t sender_id, int16_t rssi_2)
{
	// uint8_t pkt_type = data[0];
}

void sensor_main(void)
{
	int err;

	sensor_init();

	while (1) {
		err = receive(1, 30);
		if (err) {
			LOG_ERR("Reception failed, err %d", err);
			return;
		}

		k_sem_take(&operation_sem, K_FOREVER);

		struct rx_data_item rx_item;
		int rx_count = 0;

		while (rx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
		       rx_queue_get(&rx_item, K_NO_WAIT) == 0) {
			sensor_process_rx(rx_item.data, rx_item.sender_id, rx_item.rssi_2);
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

	}
}
