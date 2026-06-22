/*
+------------------------------------------------------------------------------+
| IRISS Inc.
| -----------------------------------------------------------------------------
| Copyright 2023-2025 (c) IRISS Inc. All rights reserved.
+------------------------------------------------------------------------------+
*/
/**
 *  @file    slm_at_main.c
 *  @brief   AT command server (literal port of the h745 design pattern).
 *
 *  Wire format (h745 convention — '#' prefix, not '+'):
 *      Host -> device:   AT#CMD?<CR><LF>
 *                        AT#DATAINIT="<SN>","<TYPE>","<ID>","<LEN>","<CRC>"<CR><LF>
 *                        AT#DATA="<ID>","<PAGE>","<CHUNK>","<CRC>","<DATA>"<CR><LF>
 *      Device -> host:   <CR><LF>#CMD: value<CR><LF>
 *                        <CR><LF>OK<CR><LF>
 *                        <CR><LF>ERROR<CR><LF>
 *
 *  Dispatch is a flat strstr() chain — there is no dispatch table, matching
 *  the h745 codebase style. Each handler builds its response with snprintf()
 *  and pushes it via slm_at_tx_write().
 */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(slm_at_main, CONFIG_MAIN_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "slm_at_main.h"
#include "slm_at_uart.h"
#include "slm_at_sub.h"

#include "product_info.h"
#include "radio.h"
#include "storage.h"
#include "config.h"
#include "mesh.h"
#include "data.h"
#include "psram.h"
#include "protocol.h"
#include "large_data.h"


/* ---- Public response strings (extern'd from other modules if needed). ---- */
const char AT_RESP_OK[]    = {"\r\nOK\r\n"};
const char AT_RESP_ERROR[] = {"\r\nERROR\r\n"};

static at_pending_ack_t pending_ack = {
    .data_type = 0,
    .sn = 0,
    .id = 0,
    .page = 0,
    .chunk = 0,
};

/* ---- Local helpers ---------------------------------------------------- */

void at_send_line(const char *s)
{
    char buf[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    int  n = snprintf(buf, sizeof(buf), "\r\n%s\r\n", s);
    if (n > 0) {
        (void)slm_at_tx_write((const uint8_t *)buf, (size_t)n, false);
    }
}

void at_ok(void)
{
    (void)slm_at_tx_write((const uint8_t *)AT_RESP_OK,
                          strlen(AT_RESP_OK), false);
}

void at_error(void)
{
    (void)slm_at_tx_write((const uint8_t *)AT_RESP_ERROR,
                          strlen(AT_RESP_ERROR), false);
}

static const char *device_type_name(void)
{
    switch (get_device_type()) {
    case DEVICE_TYPE_GATEWAY: return "GATEWAY";
    case DEVICE_TYPE_ANCHOR:  return "ANCHOR";
    case DEVICE_TYPE_SENSOR:  return "SENSOR";
    default:                  return "UNKNOWN";
    }
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Locate the idx-th double-quoted field in 'args'. On success returns a
 * pointer to the first char inside the quotes and writes its length (chars
 * between the quotes, may be 0) to *len_out. Returns NULL if there is no
 * idx-th complete "..." field. */
const char *field_get(const char *args, unsigned idx, size_t *len_out)
{
    const char *p = args;
    for (unsigned n = 0; ; n++) {
        const char *open = strchr(p, '"');
        if (open == NULL) {
            return NULL;
        }
        const char *close = strchr(open + 1, '"');
        if (close == NULL) {
            return NULL;
        }
        if (n == idx) {
            *len_out = (size_t)(close - (open + 1));
            return open + 1;
        }
        p = close + 1;
    }
}

/* Extract the idx-th quoted field and parse its contents as a big-endian hex
 * number into *out. The field must be non-empty and all hex. */
int field_hex_u64(const char *args, unsigned idx, uint64_t *out)
{
    size_t      len;
    const char *f = field_get(args, idx, &len);
    if (f == NULL || len == 0) {
        return -EINVAL;
    }
    uint64_t v = 0;
    for (size_t i = 0; i < len; i++) {
        int nib = hex_nibble(f[i]);
        if (nib < 0) {
            return -EINVAL;
        }
        v = (v << 4) | (uint8_t)nib;
    }
    *out = v;
    return 0;
}

/* ---- Hex helper for the AT#DATA payload --------------------- */

int hex_decode(const char *hex, size_t hex_len, uint8_t *out, size_t out_max)
{
    if ((hex_len % 2) != 0) {
        return -EINVAL;
    }
    size_t bytes = hex_len / 2;
    if (bytes > out_max) {
        return -EINVAL;
    }
    for (size_t i = 0; i < bytes; i++) {
        int hi = hex_nibble(hex[i * 2]);
        int lo = hex_nibble(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0) {
            return -EINVAL;
        }
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return (int)bytes;
}

void set_at_pending_ack(at_pending_ack_t ack)
{
    pending_ack = ack;
}

/* ---- Command handlers ------------------------------------------------ */

static void cmd_at(void)
{
    at_ok();
}

static void cmd_get_version(void)
{
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#VERSION: %d", FIRMWARE_VERSION);
    at_send_line(resp);
    at_ok();
}

static void cmd_get_devtype(void)
{
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#DEVTYPE: %s", device_type_name());
    at_send_line(resp);
    if (get_device_type() > 3 || get_device_type() == 0) {
        at_error();
    } else {
        at_ok();
    }
}

static void cmd_get_devid(void)
{
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#DEVID: %u", get_device_id());
    at_send_line(resp);
    if (get_device_id() == 0xFFFF || get_device_id() == 0) {
        at_error();
    } else {
        at_ok();
    }
}

static void cmd_get_sn(void)
{
    char     resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    uint64_t sn = get_serial_number();
    snprintf(resp, sizeof(resp), "#SN: 0x%08x%08x",
             (unsigned)(sn >> 32), (unsigned)(sn & 0xFFFFFFFF));
    at_send_line(resp);
    if (get_serial_number() == 0xFFFFFFFFFFFFFFFF || get_serial_number() == 0) {
        at_error();
    } else {
        at_ok();
    }
}

static void cmd_get_hop(void)
{
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#HOP: %u", get_hop_number());
    at_send_line(resp);
    if (get_hop_number() == 0xFF) {
        at_error();
    } else {
        at_ok();
    }
}

static void cmd_set_sn(const char *sn_str)
{
    if (sn_str == NULL) {
        at_error();
        return;
    }
    uint64_t sn;
    if (field_hex_u64(sn_str, 0, &sn) != 0) {
        at_error();
        return;
    }
    if (sn == 0 || sn == 0xFFFFFFFFFFFFFFFF) {
        at_error();
        return;
    }

    set_serial_number(sn);
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#SN: 0x%08x%08x",
             (unsigned)(sn >> 32), (unsigned)(sn & 0xFFFFFFFF));
    at_send_line(resp);
    at_ok();
}

static void cmd_reboot(void)
{
    at_ok();
    k_msleep(50);
    sys_reboot(SYS_REBOOT_COLD);
}


/* ---- Dispatch ------------------------------------------------------- */

/* Flat strstr() chain — no dispatch table, matching the h745 codebase. */
static void dispatch(char *line)
{
    if (strcmp(line, "OK") == 0) {
        switch (pending_ack.data_type) {
            case DATA_TYPE_REPORT:
            {
                uint16_t device_id = find_device_id_by_sn(pending_ack.sn);
                (void)report_slot_release_by_id(device_id, pending_ack.id, true);
            }
            break;

            case DATA_TYPE_CONFIG:
            {
                (void)config_slot_release_by_id(pending_ack.id, STATUS_SUCCESS);
            }
            break;

            case DATA_TYPE_LARGE_DATA:
            {
                if (pending_ack.page == -1 && pending_ack.chunk == -1) {
                    (void)cmd_ld_init_ack(NULL, true); // 'p' should be defined as the pointer to the "#LDINITACK" substring in 'line'
                } else {
                    (void)cmd_ld_chunk_ack(NULL, true); // 'p' should be defined as the pointer to the "#LDACK" substring in 'line'
                }
            }
            break;

            case DATA_TYPE_OTA:
            {

            }
            break;

            default:
            {
                LOG_WRN("Received OK for unknown pending ack type %u", pending_ack.data_type);
            }
            break;
        }
        /* Consume the pending ack so a later spurious OK doesn't re-fire. */
        pending_ack.data_type = 0;
        return;
    }

    if (strcmp(line, "ERROR") == 0) {
        switch (pending_ack.data_type) {
            case DATA_TYPE_REPORT:
            {
                uint16_t device_id = find_device_id_by_sn(pending_ack.sn);
                (void)report_slot_release_by_id(device_id, pending_ack.id, false);
            }
            break;

            case DATA_TYPE_CONFIG:
            {
                (void)config_slot_release_by_id(pending_ack.id, false);
            }
            break;

            case DATA_TYPE_LARGE_DATA:
            {

            }
            break;

            case DATA_TYPE_OTA:
            {

            }
            break;

            default:
            {
                LOG_WRN("Received ERROR for unknown pending ack type %u", pending_ack.data_type);
            }
            break;
        }
        /* Consume the pending ack so a later spurious OK doesn't re-fire. */
        pending_ack.data_type = 0;
        return;
    }

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        {
            if (strstr(line, "#REPORT") != NULL) {
                char *p = strstr(line, "#REPORT");
                cmd_report_ack(p);
                return;
            } else if (strstr(line, "AT#CONFIG") != NULL) {
                char *p = strstr(line, "AT#CONFIG") + strlen("AT#CONFIG");
                cmd_config(p);
                return;
            } else if (strstr(line, "#LDINIT") != NULL) {
                char *p = strstr(line, "#LDINIT");
                cmd_ld_init_ack(p, false);
                return;
            } else if (strstr(line, "#LD") != NULL) {
                char *p = strstr(line, "#LD");
                cmd_ld_chunk_ack(p, false);
                return;
            }
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {

        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            if (strstr(line, "AT#REPORT") != NULL) {
                char *p = strstr(line, "AT#REPORT") + strlen("AT#REPORT");
                cmd_report(p);
                return;
            } else if (strstr(line, "#CONFIG") != NULL) {
                char *p = strstr(line, "#CONFIG");
                cmd_config_ack(p);
                return;
            } else if (strstr(line, "AT#LDINIT") != NULL) {
                char *p = strstr(line, "AT#LDINIT") + strlen("AT#LDINIT");
                cmd_ld_init(p);
                return;
            } else if (strstr(line, "AT#LD") != NULL) {
                char *p = strstr(line, "AT#LD") + strlen("AT#LD");
                cmd_ld_chunk(p);
                return;
            }
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any AT command if this device has invalid type
            at_error();
            return;
        }
    }

    /* h745 commands are case-sensitive; we accept exactly that. */
    if (strstr(line, "AT#VERSION?") != NULL) {
        cmd_get_version();
    } else if (strstr(line, "AT#DEVTYPE?") != NULL) {
        cmd_get_devtype();
    } else if (strstr(line, "AT#DEVID?") != NULL) {
        cmd_get_devid();
    } else if (strstr(line, "AT#SN?") != NULL) {
        cmd_get_sn();
    } else if (strstr(line, "AT#HOP?") != NULL) {
        cmd_get_hop();
    } else if (strstr(line, "AT#SN") != NULL) {
        char *p = strstr(line, "AT#SN") + strlen("AT#SN");
        cmd_set_sn(p);
    } else if (strstr(line, "AT#REBOOT") != NULL) {
        cmd_reboot();
    } else if (strstr(line, "AT") != NULL) {
        char *at = strstr(line, "AT");
        char  next = at[2];
        if (next == '\r' || next == '\n' || next == '\0') {
            cmd_at();
        } else {
            at_error();
        }
    } else {
        at_error();
    }
}

/* ---- Run cycle ------------------------------------------------------ */

int slm_at_init(void)
{
    return slm_at_uart_init();
}

void slm_at_run_cycle(void)
{
    char buf[SLM_UART_AT_COMMAND_LEN];

    /* Non-blocking peek: if no message is ready, return immediately so the
     * outer state machine can do other work. */
    if (!slm_at_wait_for_message(0)) {
        return;
    }

    slm_at_get_message_copy(buf);

    /* The UART consumer merges everything queued in the ring buffer into one
     * NUL-terminated chunk, so a single message may contain multiple lines
     * (e.g. a downstream peer replying '#CONFIG: ...\r\nOK\r\n'). Split on
     * CR/LF and dispatch each non-empty line in order. */
    char *p = buf;
    while (*p) {
        /* Skip leading CR/LF separators. */
        while (*p == '\r' || *p == '\n') {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        /* Find the end of this line. */
        char *end = p;
        while (*end != '\0' && *end != '\r' && *end != '\n') {
            end++;
        }
        /* NUL-terminate this line so dispatch() sees just it. */
        char saved = *end;
        *end = '\0';
        dispatch(p);
        /* Restore (not strictly needed, but keeps buf re-walkable). */
        *end = saved;
        p = (saved == '\0') ? end : end + 1;
    }
}
