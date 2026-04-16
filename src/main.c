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
#include "gateway.h"
#include "anchor.h"
#include "sensor.h"
#include "testing/factory_reset.h"

LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

BUILD_ASSERT(CONFIG_CARRIER, "Carrier must be configured according to local regulations");

int main(void)
{
	int err;

	LOG_INF("Dect NR+ PHY mesh started");

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

	/* Check for factory reset (hold Button 1 for 3s at boot). */
	factory_reset_init();

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

	uint16_t hw_id;

	hwinfo_get_device_id((void *)&hw_id, sizeof(hw_id));
	radio_set_device_id(hw_id);

	LOG_INF("Dect NR+ PHY initialized, device ID: %d", hw_id);

	err = nrf_modem_dect_phy_capability_get();
	if (err) {
		LOG_ERR("nrf_modem_dect_phy_capability_get failed, err %d", err);
	}

	switch (PRODUCT_DEVICE_TYPE) {
	case DEVICE_TYPE_GATEWAY:
		gateway_main();
		break;
	case DEVICE_TYPE_ANCHOR:
		anchor_main();
		break;
	case DEVICE_TYPE_SENSOR:
		sensor_main();
		break;
	default:
		LOG_ERR("Unable to detect device type");
		break;
	}

	return 0;
}
