/*
 * Runtime status LED handling for LED1 (devicetree alias led1).
 *
 * A dedicated low-priority thread watches the device's own hop number.
 * While hop_num == 0xFF (no route to the gateway) LED1 blinks; for any
 * other hop_num LED1 is held OFF.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "led.h"
#include "../product_info.h"

LOG_MODULE_REGISTER(led, CONFIG_MAIN_LOG_LEVEL);

#define LED1_NODE DT_ALIAS(led1)

#define LED_BLINK_PERIOD_MS   250   /* toggle interval while blinking */
#define NO_ROUTE_HOP_NUM      0xFF

#define LED_THREAD_STACK_SIZE 1024
#define LED_THREAD_PRIO       7

static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

K_THREAD_STACK_DEFINE(led_thread_stack, LED_THREAD_STACK_SIZE);
static struct k_thread led_thread_data;

static void led_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	bool led_on = false;

	while (1) {
		if (get_hop_number() == NO_ROUTE_HOP_NUM) {
			/* No route to gateway: blink. */
			led_on = !led_on;
			gpio_pin_set_dt(&led1, led_on);
			k_msleep(LED_BLINK_PERIOD_MS);
		} else {
			/* Route established: hold OFF. */
			if (led_on) {
				led_on = false;
				gpio_pin_set_dt(&led1, 0);
			}
			k_msleep(LED_BLINK_PERIOD_MS);
		}
	}
}

int led_init(void)
{
	if (!gpio_is_ready_dt(&led1)) {
		LOG_ERR("LED1 GPIO not ready");
		return -ENODEV;
	}

	int err = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
	if (err) {
		LOG_ERR("LED1 configure failed, err %d", err);
		return err;
	}

	k_thread_create(&led_thread_data, led_thread_stack,
			K_THREAD_STACK_SIZEOF(led_thread_stack),
			led_thread, NULL, NULL, NULL,
			LED_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&led_thread_data, "led");

	LOG_INF("Status LED initialized");
	return 0;
}
