/*
 * AT command host interface.
 *
 * Interrupt-driven UART RX assembles a line into rx_line[]; the line is
 * pushed to a message queue and a worker thread parses & dispatches.
 * Keeping parse/dispatch off the ISR avoids touching radio/storage code
 * in interrupt context.
 */

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include "serial.h"
#include "product_info.h"
#include "radio.h"

LOG_MODULE_REGISTER(serial, CONFIG_LOG_DEFAULT_LEVEL);

#define AT_LINE_MAX 128
#define AT_RESP_MAX 128
#define AT_THREAD_STACK 2048
#define AT_THREAD_PRIO  7

static const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));

/* RX line assembly buffer (ISR-side). */
static char rx_line[AT_LINE_MAX];
static size_t rx_len;

/* Echo typed characters back to the host (default on, like a real AT modem).
 * Toggle via ATE0 / ATE1. */
static volatile bool echo_enabled = true;

/* Completed lines queued for the worker thread. */
struct at_line_msg {
	char line[AT_LINE_MAX];
};
K_MSGQ_DEFINE(at_lineq, sizeof(struct at_line_msg), 4, 4);

/* ---- TX helpers (worker-thread context) ---- */

static void uart_write_str(const char *s)
{
	while (*s) {
		uart_poll_out(uart_dev, *s++);
	}
}

static void at_send_line(const char *s)
{
	uart_write_str("\r\n");
	uart_write_str(s);
	uart_write_str("\r\n");
}

static void at_ok(void)
{
	at_send_line("OK");
}

static void at_error(void)
{
	at_send_line("ERROR");
}

/* ---- Command handlers ---- */

static const char *device_type_name(void)
{
	switch (PRODUCT_DEVICE_TYPE) {
	case DEVICE_TYPE_GATEWAY: return "GATEWAY";
	case DEVICE_TYPE_ANCHOR:  return "ANCHOR";
	case DEVICE_TYPE_SENSOR:  return "SENSOR";
	default:                  return "UNKNOWN";
	}
}

static void cmd_at(const char *args)
{
	(void)args;
	at_ok();
}

static void cmd_version(const char *args)
{
	(void)args;
	char resp[AT_RESP_MAX];
	snprintf(resp, sizeof(resp), "+VERSION: %d", FIRMWARE_VERSION);
	at_send_line(resp);
	at_ok();
}

static void cmd_devtype(const char *args)
{
	(void)args;
	char resp[AT_RESP_MAX];
	snprintf(resp, sizeof(resp), "+DEVTYPE: %s", device_type_name());
	at_send_line(resp);
	at_ok();
}

static void cmd_devid(const char *args)
{
	(void)args;
	char resp[AT_RESP_MAX];
	snprintf(resp, sizeof(resp), "+DEVID: %u", radio_get_device_id());
	at_send_line(resp);
	at_ok();
}

static void cmd_sn(const char *args)
{
	(void)args;
	char resp[AT_RESP_MAX];
	uint64_t sn = PRODUCT_SERIAL_NUMBER;
	snprintf(resp, sizeof(resp), "+SN: 0x%08x%08x",
		 (unsigned)(sn >> 32), (unsigned)(sn & 0xFFFFFFFF));
	at_send_line(resp);
	at_ok();
}

static void cmd_hop(const char *args)
{
	(void)args;
	char resp[AT_RESP_MAX];
	snprintf(resp, sizeof(resp), "+HOP: %u", PRODUCT_HOP_NUMBER);
	at_send_line(resp);
	at_ok();
}

static void cmd_reboot(const char *args)
{
	(void)args;
	at_ok();
	/* Give the UART a moment to drain before resetting. */
	k_msleep(50);
	sys_reboot(SYS_REBOOT_COLD);
}

static void cmd_echo_off(const char *args)
{
	(void)args;
	echo_enabled = false;
	at_ok();
}

static void cmd_echo_on(const char *args)
{
	(void)args;
	echo_enabled = true;
	at_ok();
}

/* Dispatch table.
 *
 * Match against everything after "AT" (so "AT" itself is the empty key).
 * For query commands include the trailing '?'. The match is case-insensitive
 * and exact (no prefix matching), to avoid surprises like AT+VER matching
 * AT+VERSION? */
struct at_handler {
	const char *key;
	void (*fn)(const char *args);
};

static const struct at_handler handlers[] = {
	{ "",          cmd_at },
	{ "E0",        cmd_echo_off },
	{ "E1",        cmd_echo_on },
	{ "+VERSION?", cmd_version },
	{ "+DEVTYPE?", cmd_devtype },
	{ "+DEVID?",   cmd_devid },
	{ "+SN?",      cmd_sn },
	{ "+HOP?",     cmd_hop },
	{ "+REBOOT",   cmd_reboot },
};

/* ---- Parser ---- */

/* Trim leading/trailing whitespace in-place. Returns the trimmed pointer. */
static char *trim(char *s)
{
	while (*s && isspace((unsigned char)*s)) {
		s++;
	}
	char *end = s + strlen(s);
	while (end > s && isspace((unsigned char)end[-1])) {
		end--;
	}
	*end = '\0';
	return s;
}

static int strcasecmp_local(const char *a, const char *b)
{
	while (*a && *b) {
		int ca = toupper((unsigned char)*a);
		int cb = toupper((unsigned char)*b);
		if (ca != cb) {
			return ca - cb;
		}
		a++;
		b++;
	}
	return (unsigned char)*a - (unsigned char)*b;
}

static void dispatch(char *line)
{
	line = trim(line);

	/* Empty line — ignore silently (matches modem behavior). */
	if (line[0] == '\0') {
		return;
	}

	/* Must start with "AT" (case-insensitive). */
	if (!(line[0] == 'A' || line[0] == 'a') ||
	    !(line[1] == 'T' || line[1] == 't')) {
		at_error();
		return;
	}

	const char *suffix = line + 2;

	for (size_t i = 0; i < ARRAY_SIZE(handlers); i++) {
		if (strcasecmp_local(suffix, handlers[i].key) == 0) {
			handlers[i].fn(NULL);
			return;
		}
	}

	at_error();
}

/* ---- ISR ---- */

static void uart_isr(const struct device *dev, void *user_data)
{
	(void)user_data;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!uart_irq_rx_ready(dev)) {
			continue;
		}

		uint8_t byte;
		while (uart_fifo_read(dev, &byte, 1) == 1) {
			/* Backspace (0x08) or DEL (0x7F): erase last buffered char
			 * and visually erase on the terminal with "\b \b". */
			if (byte == 0x08 || byte == 0x7F) {
				if (rx_len > 0) {
					rx_len--;
					if (echo_enabled) {
						uart_poll_out(dev, '\b');
						uart_poll_out(dev, ' ');
						uart_poll_out(dev, '\b');
					}
				}
				continue;
			}

			/* End of line — echo CRLF, then push to the queue. */
			if (byte == '\r' || byte == '\n') {
				if (echo_enabled) {
					uart_poll_out(dev, '\r');
					uart_poll_out(dev, '\n');
				}
				if (rx_len == 0) {
					continue; /* ignore blank lines */
				}
				rx_line[rx_len] = '\0';

				struct at_line_msg msg;
				memcpy(msg.line, rx_line, rx_len + 1);
				/* If the queue is full, drop — host will time out and retry. */
				(void)k_msgq_put(&at_lineq, &msg, K_NO_WAIT);

				rx_len = 0;
				continue;
			}

			/* Buffer overflow — reset and ignore this line entirely. */
			if (rx_len >= sizeof(rx_line) - 1) {
				rx_len = 0;
				continue;
			}

			rx_line[rx_len++] = (char)byte;

			/* Echo printable characters back so the user sees what they type. */
			if (echo_enabled && byte >= 0x20 && byte < 0x7F) {
				uart_poll_out(dev, byte);
			}
		}
	}
}

/* ---- Worker thread ---- */

static void at_worker(void *a, void *b, void *c)
{
	(void)a; (void)b; (void)c;

	struct at_line_msg msg;
	while (1) {
		if (k_msgq_get(&at_lineq, &msg, K_FOREVER) == 0) {
			dispatch(msg.line);
		}
	}
}

K_THREAD_DEFINE(at_worker_tid, AT_THREAD_STACK, at_worker,
		NULL, NULL, NULL,
		AT_THREAD_PRIO, 0, K_TICKS_FOREVER);

int at_command_init(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("AT UART not ready");
		return -ENODEV;
	}

	uart_irq_callback_set(uart_dev, uart_isr);
	uart_irq_rx_enable(uart_dev);

	k_thread_start(at_worker_tid);

	LOG_INF("AT command interface ready");
	return 0;
}
