/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>
#include "radio.h"
#include "queue.h"
#include "product_info.h"
#include "mesh.h"

LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

BUILD_ASSERT(CONFIG_CARRIER, "Carrier must be configured according to local regulations");

int main(void)
{
	int err;
	uint32_t tx_handle = 0;
	uint32_t rx_handle = 1;

	LOG_INF("Dect NR+ PHY mesh started");

	err = product_info_init();
	if (err) {
		LOG_ERR("product_info_init failed, err %d", err);
		return err;
	}

	err = nrf_modem_lib_init();
	if (err) {
		LOG_ERR("modem init failed, err %d", err);
		return err;
	}

	err = nrf_modem_dect_phy_event_handler_set(dect_phy_event_handler);
	if (err) {
		LOG_ERR("nrf_modem_dect_phy_event_handler_set failed, err %d", err);
		return err;
	}

	err = nrf_modem_dect_phy_init();
	if (err) {
		LOG_ERR("nrf_modem_dect_phy_init failed, err %d", err);
		return err;
	}

	k_sem_take(&operation_sem, K_FOREVER);
	if (radio_exit) {
		return -EIO;
	}

	err = nrf_modem_dect_phy_configure(&dect_phy_config_params);
	if (err) {
		LOG_ERR("nrf_modem_dect_phy_configure failed, err %d", err);
		return err;
	}

	k_sem_take(&operation_sem, K_FOREVER);
	if (radio_exit) {
		return -EIO;
	}

	err = nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
	if (err) {
		LOG_ERR("nrf_modem_dect_phy_activate failed, err %d", err);
		return err;
	}

	k_sem_take(&operation_sem, K_FOREVER);
	if (radio_exit) {
		return -EIO;
	}

	radio_set_device_id(device_id);

	LOG_INF("Dect NR+ PHY initialized, device ID: %d", device_id);

	err = nrf_modem_dect_phy_capability_get();
	if (err) {
		LOG_ERR("nrf_modem_dect_phy_capability_get failed, err %d", err);
	}

	#define MAX_PROCESS_PER_CYCLE 4

	while (1) {
		/* RX window. */
		err = receive(rx_handle, 30);
		if (err) {
			LOG_ERR("Reception failed, err %d", err);
			return err;
		}

		k_sem_take(&operation_sem, K_FOREVER);

		/* Process up to 4 received messages per cycle. */
		struct rx_data_item rx_item;
		int rx_count = 0;

		while (rx_count < MAX_PROCESS_PER_CYCLE &&
		       rx_queue_get(&rx_item, K_NO_WAIT) == 0) {
			LOG_INF("From device %d (RSSI: %d.%d)", rx_item.sender_id, (rx_item.rssi_2 / 2), (rx_item.rssi_2 & 0b1) * 5);
			rx_count++;
		}

		/* Transmit up to 4 queued packets per cycle (high priority first). */
		struct tx_data_item tx_item;
		int tx_count = 0;

		while (tx_count < MAX_PROCESS_PER_CYCLE &&
		       tx_queue_get(&tx_item, K_NO_WAIT) == 0) {
			err = transmit(tx_handle, tx_item.data, tx_item.data_len);
			if (err) {
				LOG_ERR("TX queue transmit failed, err %d", err);
				break;
			}
			k_sem_take(&operation_sem, K_FOREVER);
			tx_count++;
		}
	}

	return 0;
}
