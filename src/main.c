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
#include <zephyr/drivers/hwinfo.h>
#include "radio.h"
#include "product_info.h"
#include "storage.h"
#include "main_sub.h"
#include "psram.h"
#include "slm_at_main.h"
#include "config.h"
#include "data.h"
#include "large_data.h"
#include "mesh.h"
#include "tracker.h"
#include "testing/buttons.h"
#include "testing/led.h"

LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

BUILD_ASSERT(CONFIG_CARRIER, "Carrier must be configured according to local regulations");

int main(void)
{
	int err;

	LOG_INF("---****--- Dect NR+ PHY mesh started ---****---");

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

	err = nrf_modem_dect_phy_capability_get();
	if (err) {
		LOG_ERR("nrf_modem_dect_phy_capability_get failed, err %d", err);
	}

	err = product_info_init();
	if (err) {
		LOG_ERR("product_info_init failed, err %d", err);
		return err;
	}

	err = storage_init();
	if (err) {
		LOG_ERR("storage_init failed, err %d", err);
		return err;
	}

	err = psram_init();
	if (err) {
		LOG_WRN("PSRAM init failed, err %d (tracker will not work)", err);
	}

	err = slm_at_init();
	if (err) {
		LOG_WRN("SLM AT init failed, err %d", err);
	}

	/* Testing Purpose Only*/
	buttons_init();
	led_init();

	while (1) {
		// Main Sub State Machine
		main_sub_run();
		// SLM AT Command Processor
		slm_at_run_cycle();
	}

	return 0;
}
