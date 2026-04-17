/*
 * Product info — reads GPIO straps to determine device type at runtime.
 *
 * Pin mapping (active-low, internal pull-up):
 *   P0.21  P0.23  →  type
 *     0      0       Gateway
 *     0      1       Anchor
 *     1      0       Sensor
 *     1      1       Sensor (reserved, defaults to Sensor)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/logging/log.h>
#include "product_info.h"
#include "storage.h"

LOG_MODULE_REGISTER(product_info, CONFIG_PRODUCT_INFO_LOG_LEVEL);

#define DEVTYPE_PIN0_NODE DT_NODELABEL(devtype_pin0)
#define DEVTYPE_PIN1_NODE DT_NODELABEL(devtype_pin1)

static const struct gpio_dt_spec pin0 = GPIO_DT_SPEC_GET(DEVTYPE_PIN0_NODE, gpios);
static const struct gpio_dt_spec pin1 = GPIO_DT_SPEC_GET(DEVTYPE_PIN1_NODE, gpios);

device_type_t PRODUCT_DEVICE_TYPE;
uint64_t PRODUCT_SERIAL_NUMBER;
uint8_t PRODUCT_HOP_NUMBER;

int product_info_init(void)
{
	int err;
	int bit0, bit1;

	/* Configure GPIO pins as inputs. */
	if (!gpio_is_ready_dt(&pin0) || !gpio_is_ready_dt(&pin1)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&pin0, GPIO_INPUT);
	if (err) {
		LOG_ERR("Failed to configure pin0, err %d", err);
		return err;
	}

	err = gpio_pin_configure_dt(&pin1, GPIO_INPUT);
	if (err) {
		LOG_ERR("Failed to configure pin1, err %d", err);
		return err;
	}

	/* Read pin values (active-low: 1 = grounded, 0 = floating). */
	bit0 = gpio_pin_get_dt(&pin0);
	bit1 = gpio_pin_get_dt(&pin1);

	if (bit0 < 0 || bit1 < 0) {
		LOG_ERR("Failed to read GPIO pins");
		return -EIO;
	}

	/* Decode device type from pin combination. */
	uint8_t code = (bit0 << 1) | bit1;

	switch (code) {
	case 0x00:
		PRODUCT_DEVICE_TYPE = DEVICE_TYPE_GATEWAY;
		break;
	case 0x01:
		PRODUCT_DEVICE_TYPE = DEVICE_TYPE_ANCHOR;
		break;
	case 0x02:
	case 0x03:
	default:
		PRODUCT_DEVICE_TYPE = DEVICE_TYPE_SENSOR;
		break;
	}

	/* Build serial number: 0x00{device_id_16}00DEADBEEF */
	uint16_t hw_id;

	hwinfo_get_device_id((void *)&hw_id, sizeof(hw_id));
	PRODUCT_SERIAL_NUMBER = ((uint64_t)hw_id << 40) | 0x00DEADBEEFULL;

	/* Set initial hop number based on device type. */
	switch (PRODUCT_DEVICE_TYPE) {
	case DEVICE_TYPE_GATEWAY:
		PRODUCT_HOP_NUMBER = 0;
		break;
	case DEVICE_TYPE_SENSOR:
		PRODUCT_HOP_NUMBER = 0xFF;  /* sensors don't have hop */
		break;
	default:
		PRODUCT_HOP_NUMBER = 0xFF;  /* anchor: updated after pairing */
		break;
	}

	LOG_INF("Device type: %s, SN: 0x%016llx, Hop: %d",
		device_type_str(PRODUCT_DEVICE_TYPE),
		PRODUCT_SERIAL_NUMBER, PRODUCT_HOP_NUMBER);

	return 0;
}

void product_info_update_hop(void)
{
	if (PRODUCT_DEVICE_TYPE != DEVICE_TYPE_ANCHOR) {
		return;
	}

	int infra = storage_infra_count();

	if (infra == 0) {
		PRODUCT_HOP_NUMBER = 0xFF;
		return;
	}

	/* Find minimum hop among infra entries. */
	uint8_t min_hop = 0xFF;

	for (int i = 0; i < infra; i++) {
		infra_entry_t entry;

		if (storage_infra_get(i, &entry) == 0 && entry.hop_num < min_hop) {
			min_hop = entry.hop_num;
		}
	}

	PRODUCT_HOP_NUMBER = (min_hop < 0xFE) ? min_hop + 1 : 0xFF;

	LOG_INF("Anchor hop updated: %d (min infra hop: %d)", PRODUCT_HOP_NUMBER, min_hop);
}
