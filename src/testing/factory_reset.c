/*
 * Factory reset via Button 1 — testing/dev only.
 *
 * At boot, checks if Button 1 is pressed within 1 second.
 * If pressed:
 *   1. "Button 1 detected - keep holding for factory reset..."
 *   2. Waits 3 seconds, checks still held
 *   3. "Factory Reset....."
 *   4. Waits 2 seconds (for user to release button)
 *   5. Clears all partitions and reboots
 *
 * To remove: delete this file and factory_reset.h,
 * remove factory_reset_init() from main.c.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include "../storage.h"
#include "factory_reset.h"

LOG_MODULE_REGISTER(factory_reset, CONFIG_MAIN_LOG_LEVEL);

#define DETECT_WINDOW_MS  1000
#define RESET_HOLD_MS     3000
#define POLL_INTERVAL_MS  100

#define BUTTON_NODE DT_NODELABEL(button0)

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

int factory_reset_init(void)
{
	if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("Button GPIO not ready");
		return -ENODEV;
	}

	int err = gpio_pin_configure_dt(&button, GPIO_INPUT);

	if (err) {
		LOG_ERR("Failed to configure button, err %d", err);
		return err;
	}

	/* Check if button is pressed within 1 second of boot. */
	bool detected = false;

	for (int elapsed = 0; elapsed < DETECT_WINDOW_MS; elapsed += POLL_INTERVAL_MS) {
		if (gpio_pin_get_dt(&button)) {
			detected = true;
			break;
		}
		k_msleep(POLL_INTERVAL_MS);
	}

	if (!detected) {
		return 0;
	}

	LOG_WRN("Button 1 detected - keep holding for factory reset...");

	/* Wait 3 seconds, check if still held. */
	int held_ms = 0;

	while (held_ms < RESET_HOLD_MS) {
		k_msleep(POLL_INTERVAL_MS);
		if (gpio_pin_get_dt(&button) == 0) {
			LOG_INF("Button released, factory reset cancelled");
			return 0;
		}
		held_ms += POLL_INTERVAL_MS;
	}

	LOG_WRN("Factory Reset.....");

	/* Wait 2 seconds for user to release button. */
	k_msleep(2000);

	storage_infra_clear();
	storage_sensor_clear();
	storage_mesh_clear();

	LOG_WRN("Factory Reset Complete, Rebooting...");
	k_msleep(500);
	sys_reboot(SYS_REBOOT_COLD);

	return 0;
}
