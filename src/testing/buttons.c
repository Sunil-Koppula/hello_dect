/*
 * Runtime button handling for DK button1 and button2.
 *
 * Each button has a GPIO interrupt that posts a press event to a message
 * queue. A dedicated low-priority thread drains the queue, applies a
 * debounce delay, and invokes the registered handler.
 *
 * Press is detected on release (rising edge for active-low buttons).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include "buttons.h"
#include "../product_info.h"
#include "../data.h"
#include "../large_data.h"
#include "../config.h"
#include "../psram.h"
#include "../radio.h"
#include "../protocol.h"
#include "../storage.h"
#include "../mesh.h"
#include "../mesh_layers/mesh_routing.h"

LOG_MODULE_REGISTER(buttons, CONFIG_MAIN_LOG_LEVEL);

#define BUTTON0_NODE DT_NODELABEL(button0)
#define BUTTON1_NODE DT_NODELABEL(button1)
#define BUTTON2_NODE DT_NODELABEL(button2)
#define BUTTON3_NODE DT_NODELABEL(button3)

#define DEBOUNCE_MS         50
#define BUTTON_COUNT        4
#define EVENT_QUEUE_DEPTH   8

#define BUTTONS_THREAD_STACK_SIZE  4096
#define BUTTONS_THREAD_PRIO        7

struct button_entry {
	const struct gpio_dt_spec spec;
	struct gpio_callback cb;
	button_handler_t handler;
	int64_t last_press_ms;
	int idx;          /* user-facing index: 1, 2, 3, or 4 */
};

typedef enum
{
    SENSOR_REPORT_CONFIG_FLAG_DEFAULT = 0x00, // Encrypted
    SENSOR_REPORT_CONFIG_FLAG_NOT_ENCRYPTED = (1 << 0),
    SENSOR_REPORT_CONFIG_FLAG_DEMO_MODE = (1 << 1),
    SENSOR_REPORT_CONFIG_FLAG_VALID = (1 << 2)
} sensor_report_config_flags_t;

typedef struct
{
    int8_t temperature_max1;
    int8_t temperature_min1;
    uint8_t humidity_max1;
    uint8_t humidity_min1;
    int8_t temperature_max2;
    int8_t temperature_min2;
    uint8_t humidity_max2;
    uint8_t humidity_min2;
    uint16_t ultrasound_level_max;
    uint8_t  ultrasound_center_frequency;
    uint16_t vibration_level_max;
    uint8_t  random_number;
} sensor_config_info_3105_t;

typedef struct {
	uint8_t battery_level_min;
	uint16_t sleep_time_sec;
	sensor_report_config_flags_t config_flags;
	sensor_config_info_3105_t config_info_3105;
	uint16_t config_crc16;   /* must stay last — CRC covers everything before it */
} __attribute__((packed)) sensor_config_t;

static struct button_entry buttons[BUTTON_COUNT] = {
	{ .spec = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios), .idx = 1 },
	{ .spec = GPIO_DT_SPEC_GET(BUTTON1_NODE, gpios), .idx = 2 },
	{ .spec = GPIO_DT_SPEC_GET(BUTTON2_NODE, gpios), .idx = 3 },
	{ .spec = GPIO_DT_SPEC_GET(BUTTON3_NODE, gpios), .idx = 4 },
};

K_MSGQ_DEFINE(button_events, sizeof(int), EVENT_QUEUE_DEPTH, 4);

K_THREAD_STACK_DEFINE(buttons_thread_stack, BUTTONS_THREAD_STACK_SIZE);
static struct k_thread buttons_thread_data;

static struct button_entry *find_button(int idx)
{
	for (int i = 0; i < BUTTON_COUNT; i++) {
		if (buttons[i].idx == idx) {
			return &buttons[i];
		}
	}
	return NULL;
}

static void button_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct button_entry *b = CONTAINER_OF(cb, struct button_entry, cb);
	int idx = b->idx;

	/* Best-effort enqueue; drop if queue full to avoid blocking ISR. */
	k_msgq_put(&button_events, &idx, K_NO_WAIT);
}

static void buttons_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int idx;

	while (1) {
		if (k_msgq_get(&button_events, &idx, K_FOREVER) != 0) {
			continue;
		}

		struct button_entry *b = find_button(idx);
		if (!b) {
			continue;
		}

		int64_t now = k_uptime_get();
		if (now - b->last_press_ms < DEBOUNCE_MS) {
			continue;
		}
		b->last_press_ms = now;

		if (b->handler) {
			b->handler();
		} else {
			LOG_DBG("Button %d pressed (no handler registered)", idx);
		}
	}
}

static uint32_t config_slot_psram_addr(int idx)
{
	return PSRAM_CONFIG_BASE + ((uint32_t)idx * MAX_CONFIG_SIZE);
}

int button_register_handler(int idx, button_handler_t handler)
{
	struct button_entry *b = find_button(idx);
	if (!b) {
		LOG_ERR("button_register_handler: invalid index %d", idx);
		return -EINVAL;
	}
	b->handler = handler;
	return 0;
}

static int alloc_large_data_slot(uint32_t size)
{
    int idx;
    for (idx = 0; idx < LARGE_DATA_RECEIVER_SLOT_COUNT; idx++) {
        if (!ld_slot[idx].active) {
            break;
        }
    }
    if (idx >= LARGE_DATA_RECEIVER_SLOT_COUNT) {
        LOG_WRN("alloc_large_data_slot: no free receiver slot");
        return -1;
    }
    uint16_t available_slots = 0;
    for (int i = 0; i < LARGE_DATA_SLOT_COUNT; i++) {
        if (is_ld_slot_empty[i]) {
            available_slots++;
            if (available_slots * SEND_DATA_MAX >= size) {
                // Mark these slots as used
                for (int j = i - available_slots + 1; j <= i; j++) {
                    is_ld_slot_empty[j] = false;
                }
                ld_slot[idx].active = true;
                ld_slot[idx].base_addr = LARGE_DATA_PSRAM_BASE + (i - available_slots + 1) * SEND_DATA_MAX;
                LOG_INF("Allocated large data slot %d (PSRAM 0x%06x-0x%06x) for size %u bytes", idx, ld_slot[idx].base_addr, ld_slot[idx].base_addr + available_slots * SEND_DATA_MAX - 1, size);
                return idx;
            }
        } else {
            available_slots = 0;
        }
    }
    return -1;
}

static void default_button0_handler(void)
{
	LOG_WRN("Factory Reset Button pressed");
	k_msleep(1000);
	factory_reset();

	k_msleep(500);
	sys_reboot(SYS_REBOOT_COLD);
}

static void default_button1_handler(void)
{
	if (get_device_type() == DEVICE_TYPE_SENSOR) {
		static uint16_t data_id = 0x01;
		uint16_t size = 256;
		slm_at_structure_t report = {
			.data_id = (data_id == 255) ? (data_id = 1) : data_id++, // wrap around to 1 after 255 to avoid potential overflow issues in other parts of the code
			.data_len = size,
			.device_id = get_device_id(),
			.device_type = get_device_type(),
		};

		LOG_INF("Button 1 pressed, building dummy report with size %d", size);

		uint8_t chunk[256];
		uint16_t written = 0;

		while (written < size) {
			uint16_t n = MIN((uint16_t)sizeof(chunk), (uint16_t)(size - written));
			for (uint16_t i = 0; i < n; i++) {
				chunk[i] = (uint8_t)((written + i) & 0xFF);
			}
			if (written == 0) {
				report.data_crc32 = crc32_ieee(chunk, n);
			} else {
				report.data_crc32 = crc32_ieee_update(report.data_crc32, chunk, n);
			}
			written += n;
		}

		int err = validate_at_report(&report, PACKET_PRIORITY_LOW, chunk);
		if (err) {
			LOG_ERR("validate_at_report failed (%d)", err);
		}
		return;
	}
}

static void default_button2_handler(void)
{
	LOG_WRN("Config Button pressed");
	if (get_device_type() == DEVICE_TYPE_GATEWAY) {
		int idx = 0; // For testing, always use config slot 0
		if (idx < 0) {
			LOG_WRN("No free data slot available");
			return;
		}
		config_slot[idx].active = true;
		config_slot[idx].is_sent = true;

		int err = 0;

		uint16_t dummy_id = 19152;
		uint64_t dummy_sn = ((uint64_t)dummy_id << 40) | 0x00DEADBEEFULL;

		uint16_t dst_id = 0xFFFF;
		uint8_t dst_type;
		uint8_t hop_num = 0xFF;

		for (int i = 0; i < mesh_count; i++) {
			if (mesh_devices[i].serial_num == dummy_sn) {
				dst_id = mesh_devices[i].device_id;
				dst_type = mesh_devices[i].device_type;
				hop_num = mesh_devices[i].hop_num;

				if (mesh_devices[i].device_type == DEVICE_TYPE_SENSOR) {
					for (int j = 0; j < mesh_count; j++) {
						if (mesh_devices[j].device_id == mesh_devices[i].connected_device_id) {
							hop_num = mesh_devices[j].hop_num;
							break;
						}
					}
				}
				LOG_INF("Found test device (sn = 0x%016llX) in mesh storage at index %d, device %s ID:%d hop_num:%d", mesh_devices[i].serial_num, i, device_type_str(dst_type), dst_id, hop_num);
				break;
			}
		}

		if (dst_id == 0xFFFF || hop_num == 0xFF) {
			LOG_WRN("Test device with SN 0x%016llX not found in mesh storage, cannot send config", dummy_sn);
			return;
		}

		config_slot[idx].dst_device_id = dst_id;
		config_slot[idx].dst_device_type = dst_type;

		// Generate config for testing
		sensor_config_t config = {
			.battery_level_min = 20,
			.sleep_time_sec = 60,
			.config_flags = SENSOR_REPORT_CONFIG_FLAG_DEFAULT,
			.config_info_3105 = {
				.temperature_max1 = 30,
				.temperature_min1 = 15,
				.humidity_max1 = 70,
				.humidity_min1 = 30,
				.temperature_max2 = 28,
				.temperature_min2 = 18,
				.humidity_max2 = 65,
				.humidity_min2 = 35,
				.ultrasound_level_max = 1000,
				.ultrasound_center_frequency = 40,
				.vibration_level_max = 500,
				.random_number = 0xFF,
			},
		};
		// CRC16-CCITT over everything before the trailing config_crc16 field.
		config.config_crc16 = crc16_ccitt(0xFFFF, (const uint8_t *)&config, sizeof(config) - sizeof(config.config_crc16));
		
		config_slot[idx].config_len = sizeof(config);
		config_slot[idx].config_crc32 = crc32_ieee((const uint8_t *)&config, sizeof(config));

		uint32_t addr = config_slot_psram_addr(idx);
		err = psram_write(addr, &config, sizeof(config));
		if (err) {
			LOG_ERR("psram_write @0x%06x failed (%d)", addr, err);
			config_slot[idx].active = false;
			return;
		}

		// First we have find the route
		route_discovery_t rd_pkt = {
			.device_id = config_slot[idx].dst_device_id,
			.device_type = config_slot[idx].dst_device_type,
			.hop_num = hop_num,
			.data_type = DATA_TYPE_CONFIG,
			.data_id = idx,
		};
		for (int i = 0; i < infra_count; i++) {
			send_route_discovery(&rd_pkt, infra_devices[i].entry.device_id, infra_devices[i].entry.device_type, STATUS_SUCCESS);
		}

	} else if (get_device_type() == DEVICE_TYPE_SENSOR) {
		LOG_WRN("Large Data Button pressed");
		// For testing, always use slot 0 and send large data to gateway
		uint32_t size = 100 * 1024;
		int idx = alloc_large_data_slot(size);
		if (idx < 0) {
			LOG_WRN("No free large data slot available");
			return;
		}

		ld_slot[idx].active = true;
		ld_slot[idx].upstream_ready = false;
    	ld_slot[idx].is_sent = false;
    	ld_slot[idx].is_transfered = false;
		ld_slot[idx].priority = PACKET_PRIORITY_HIGH;
		ld_slot[idx].gen_device_id = get_device_id();
		ld_slot[idx].data_id = 1; // For testing, use data_id 1
		ld_slot[idx].total_size = size;
		ld_slot[idx].page_count = (size + SEND_DATA_MAX * 20 - 1) / (SEND_DATA_MAX * 20); // Each page has 20 chunks
		ld_slot[idx].last_chunk_size = size % SEND_DATA_MAX == 0 ? SEND_DATA_MAX : size % SEND_DATA_MAX;
		ld_slot[idx].total_chunks = (size + SEND_DATA_MAX - 1) / SEND_DATA_MAX;
		ld_slot[idx].crc32 = 0; // Will be calculated in the button handler for testing
		ld_slot[idx].received_count = 0;

		uint8_t chunk[1024];
		uint32_t written = 0;

		while (written < size) {
			uint32_t n = MIN((uint32_t)sizeof(chunk), size - written);
			for (uint32_t i = 0; i < n; i++) {
				chunk[i] = (uint8_t)((written + i) & 0xFF);
			}
			if (written == 0) {
				ld_slot[idx].crc32 = crc32_ieee((const uint8_t *)chunk, n);
			} else {
				ld_slot[idx].crc32 = crc32_ieee_update(ld_slot[idx].crc32, (const uint8_t *)chunk, n);
			}
			int err = psram_write(ld_slot[idx].base_addr + written, chunk, n);
			if (err) {
				LOG_ERR("psram_write @0x%06x failed (%d)", ld_slot[idx].base_addr + written, err);
				ld_slot[idx].active = false;
				return;
			}
			written += n;
		}
		LOG_INF("Large data of size %u bytes written to PSRAM at 0x%06x for slot %d, CRC32=0x%08x", size, ld_slot[idx].base_addr, idx, ld_slot[idx].crc32);
		ld_slot[idx].upstream_ready = true;
	}
}

int buttons_init(void)
{
	button_register_handler(1, default_button0_handler);
	button_register_handler(2, default_button1_handler);
	button_register_handler(3, default_button2_handler);
	button_register_handler(4, NULL);

	for (int i = 0; i < BUTTON_COUNT; i++) {
		struct button_entry *b = &buttons[i];

		if (!gpio_is_ready_dt(&b->spec)) {
			LOG_ERR("Button %d GPIO not ready", b->idx);
			return -ENODEV;
		}

		int err = gpio_pin_configure_dt(&b->spec, GPIO_INPUT);
		if (err) {
			LOG_ERR("Button %d configure failed, err %d", b->idx, err);
			return err;
		}

		/* Trigger on release (inactive edge for active-low button). */
		err = gpio_pin_interrupt_configure_dt(&b->spec, GPIO_INT_EDGE_TO_INACTIVE);
		if (err) {
			LOG_ERR("Button %d interrupt configure failed, err %d", b->idx, err);
			return err;
		}

		gpio_init_callback(&b->cb, button_isr, BIT(b->spec.pin));
		err = gpio_add_callback(b->spec.port, &b->cb);
		if (err) {
			LOG_ERR("Button %d add_callback failed, err %d", b->idx, err);
			return err;
		}
	}

	k_thread_create(&buttons_thread_data, buttons_thread_stack,
			K_THREAD_STACK_SIZEOF(buttons_thread_stack),
			buttons_thread, NULL, NULL, NULL,
			BUTTONS_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&buttons_thread_data, "buttons");

	LOG_INF("Buttons initialized");
	return 0;
}


