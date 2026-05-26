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
 *                        AT#DATAINIT<hex><CR><LF>
 *                        AT#DATA<hex><CR><LF>
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

typedef struct {
    uint8_t data_type;
    uint16_t data_id;
    int  data_idx;
    uint8_t page;
    uint8_t chunk;
} data_info_t;

static data_info_t data_info;

#define AT_DATA_PAYLOAD_MAX 500

/* ---- Public response strings (extern'd from other modules if needed). ---- */
const char AT_RESP_OK[]    = {"\r\nOK\r\n"};
const char AT_RESP_ERROR[] = {"\r\nERROR\r\n"};

/* ---- Local helpers ---------------------------------------------------- */

static void at_send_line(const char *s)
{
    char buf[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    int  n = snprintf(buf, sizeof(buf), "\r\n%s\r\n", s);
    if (n > 0) {
        (void)slm_at_tx_write((const uint8_t *)buf, (size_t)n, false);
    }
}

static void at_ok(void)
{
    (void)slm_at_tx_write((const uint8_t *)AT_RESP_OK,
                          strlen(AT_RESP_OK), false);
}

static void at_error(void)
{
    (void)slm_at_tx_write((const uint8_t *)AT_RESP_ERROR,
                          strlen(AT_RESP_ERROR), false);
}

static const char *device_type_name(void)
{
    switch (DEVICE_TYPE) {
    case DEVICE_TYPE_GATEWAY: return "GATEWAY";
    case DEVICE_TYPE_ANCHOR:  return "ANCHOR";
    case DEVICE_TYPE_SENSOR:  return "SENSOR";
    default:                  return "UNKNOWN";
    }
}

/* ---- Data init parsing (kept compatible with the old +DATAINIT format). */

typedef struct {
    uint64_t sn;
    uint8_t  data_type;
    uint16_t data_id;
    uint16_t data_len;
    uint32_t data_crc32;
} data_init_info_t;

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int read_be(const char *hex, size_t *pos, size_t nbytes, uint64_t *out)
{
    uint64_t v = 0;
    for (size_t i = 0; i < nbytes; i++) {
        int hi = hex_nibble(hex[*pos]);
        int lo = hex_nibble(hex[*pos + 1]);
        if (hi < 0 || lo < 0) {
            return -EINVAL;
        }
        v = (v << 8) | (uint8_t)((hi << 4) | lo);
        *pos += 2;
    }
    *out = v;
    return 0;
}

/* "<sn(8)><type(1)><id(2)><len(2)><crc(4)>" — 17 bytes / 34 hex chars. */
static int parse_data_init(const char *args, data_init_info_t *out)
{
    const size_t expected_chars = (8 + 1 + 2 + 2 + 4) * 2;

    if (args == NULL || out == NULL) {
        return -EINVAL;
    }
    if (strlen(args) < expected_chars) {
        return -EINVAL;
    }

    size_t   pos = 0;
    uint64_t sn, dtype, did, dlen, crc;
    if (read_be(args, &pos, 8, &sn)    != 0) return -EINVAL;
    if (read_be(args, &pos, 1, &dtype) != 0) return -EINVAL;
    if (read_be(args, &pos, 2, &did)   != 0) return -EINVAL;
    if (read_be(args, &pos, 2, &dlen)  != 0) return -EINVAL;
    if (read_be(args, &pos, 4, &crc)   != 0) return -EINVAL;

    out->sn         = sn;
    out->data_type  = (uint8_t)dtype;
    out->data_id    = (uint16_t)did;
    out->data_len   = (uint16_t)dlen;
    out->data_crc32 = (uint32_t)crc;
    return 0;
}

static int validate_data_info(const data_init_info_t *info)
{
    if (data_info.data_type != 0) {
        LOG_WRN("Another data transfer in progress (type %u id %u), rejecting new one", data_info.data_type, data_info.data_id);
        return -EBUSY;
    }

    switch (info->data_type) {
    case DATA_TYPE_REPORT:
        return 0;

    case DATA_TYPE_CONFIG: {
        if (info->data_len > MAX_CONFIG_SIZE && info->data_len != 0) {
            return -EINVAL;
        }
        if (DEVICE_TYPE != DEVICE_TYPE_GATEWAY) {
            return -EINVAL;
        }

        uint16_t idx = 0;
        for (idx = 0; idx < mesh_count; idx++) {
            if (mesh_devices[idx].serial_num == info->sn) {
                LOG_INF("Found device SN:%llu @ mesh[%d] %s ID:%d",
                        info->sn, idx,
                        device_type_str(mesh_devices[idx].device_type),
                        mesh_devices[idx].device_id);
                break;
            }
            if (idx == mesh_count - 1) {
                LOG_INF("Device SN:%llu not in mesh", info->sn);
                return -EINVAL;
            }
        }

        int err = validate_config_slot(mesh_devices[idx].device_id,
                                       mesh_devices[idx].device_type,
                                       info->data_id, info->data_len,
                                       info->data_crc32);
        if (err < 0) {
            return -EINVAL;
        }

        // Store the info for the upcoming AT#DATA chunks to validate against and know where to route the config data when the slot is ready.
        data_info.data_type = info->data_type;
        data_info.data_id = info->data_id;
        data_info.data_idx = err;
        data_info.page = 0;
        data_info.chunk = 0;

        LOG_INF("Allocated config slot %d for %s ID:%d", err, device_type_str(mesh_devices[idx].device_type), mesh_devices[idx].device_id);
        return 0;
    }

    case DATA_TYPE_LARGE:
        return 0;

    default:
        return -EINVAL;
    }
}

/* ---- Hex helper for the AT#DATA payload --------------------- */

static int hex_decode(const char *hex, size_t hex_len, uint8_t *out, size_t out_max)
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

/* ---- Command handlers ------------------------------------------------ */

static void cmd_at(void)
{
    at_ok();
}

static void cmd_version(void)
{
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#VERSION: %d", FIRMWARE_VERSION);
    at_send_line(resp);
    at_ok();
}

static void cmd_devtype(void)
{
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#DEVTYPE: %s", device_type_name());
    at_send_line(resp);
    at_ok();
}

static void cmd_devid(void)
{
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#DEVID: %u", radio_get_device_id());
    at_send_line(resp);
    at_ok();
}

static void cmd_sn(void)
{
    char     resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    uint64_t sn = SERIAL_NUMBER;
    snprintf(resp, sizeof(resp), "#SN: 0x%08x%08x",
             (unsigned)(sn >> 32), (unsigned)(sn & 0xFFFFFFFF));
    at_send_line(resp);
    at_ok();
}

static void cmd_hop(void)
{
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#HOP: %u", DEVICE_HOP_NUMBER);
    at_send_line(resp);
    at_ok();
}

static void cmd_reboot(void)
{
    at_ok();
    k_msleep(50);
    sys_reboot(SYS_REBOOT_COLD);
}

/* AT#DATAINIT<34 hex chars> */
static void cmd_data_init(const char *args)
{
    data_init_info_t info;
    char             resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];

    if (parse_data_init(args, &info) != 0) {
        at_error();
        return;
    }
    if (validate_data_info(&info) != 0) {
        at_error();
        return;
    }

    snprintf(resp, sizeof(resp),
             "#DATAINIT: sn=%llu type=%u id=%u len=%u crc=0x%08lx",
             (unsigned long long)info.sn,
             info.data_type, info.data_id, info.data_len,
             (unsigned long)info.data_crc32);
    at_send_line(resp);
    at_ok();
}

/* AT#DATA<id:2><page:1><chunk:1><crc32:4><payload:N>
 *
 * All hex, no separators (note: no '=' after AT#DATA). Wire layout:
 *   - 8 hex chars   : header (id:2 page:1 chunk:1, big-endian)
 *   - 8 hex chars   : CRC32-IEEE of the decoded payload (big-endian)
 *   - 2*N hex chars : payload, N ≤ AT_DATA_PAYLOAD_MAX (500)
 *
 * Radio still emits SEND_DATA_MAX (180 B) per radio chunk; the firmware
 * is expected to split each AT chunk into 1..N radio chunks when forwarding.
 *
 * Chunking rules by data_type (set via AT#DATAINIT):
 *   CONFIG : exactly one chunk  → page=0, chunk=0
 *   REPORT : exactly one chunk  → page=0, chunk=0
 *   LARGE  : pages × 20 chunks  → page=0..N-1, chunk=0..19
 *
 * The 'id' field must match the most recent AT#DATAINIT. A CRC32 mismatch
 * on the chunk payload returns ERROR so the host can retry just that chunk.
 */
static void cmd_data(const char *args)
{
    /* Trim trailing CR/LF/space the parser may have left in place. */
    size_t arglen = strlen(args);
    while (arglen > 0 &&
           (args[arglen - 1] == '\r' || args[arglen - 1] == '\n' ||
            args[arglen - 1] == ' ')) {
        arglen--;
    }

    /* Minimum wire size: 8 (header) + 8 (crc) + 2 (≥1 payload byte) = 18. */
    if (arglen < 8 + 8 + 2) {
        at_error();
        return;
    }

    /* Decode the 4-byte header. */
    size_t   pos = 0;
    uint64_t v;
    if (read_be(args, &pos, 2, &v) != 0) { at_error(); return; }
    uint16_t hdr_id    = (uint16_t)v;
    if (read_be(args, &pos, 1, &v) != 0) { at_error(); return; }
    uint8_t  hdr_page  = (uint8_t)v;
    if (read_be(args, &pos, 1, &v) != 0) { at_error(); return; }
    uint8_t  hdr_chunk = (uint8_t)v;

    /* Decode the 4-byte CRC32 trailer-of-header. */
    if (read_be(args, &pos, 4, &v) != 0) { at_error(); return; }
    uint32_t hdr_crc32 = (uint32_t)v;

    /* Everything remaining is the payload. */
    size_t payload_chars = arglen - pos;
    if ((payload_chars % 2) != 0) {
        at_error();
        return;
    }
    if ((payload_chars / 2) > AT_DATA_PAYLOAD_MAX) {
        at_error();
        return;
    }

    uint8_t payload[AT_DATA_PAYLOAD_MAX];
    int     decoded = hex_decode(args + pos, payload_chars,
                                 payload, sizeof(payload));
    if (decoded < 0) {
        at_error();
        return;
    }

    /* Verify CRC32 over the decoded payload. */
    uint32_t calc_crc32 = crc32_ieee(payload, (size_t)decoded);
    if (calc_crc32 != hdr_crc32) {
        LOG_WRN("AT#DATA id=%u page=%u chunk=%u CRC mismatch: got 0x%08x, calc 0x%08x", hdr_id, hdr_page, hdr_chunk, hdr_crc32, calc_crc32);
        at_error();
        return;
    }

    /* Must have a prior AT#DATAINIT establishing the transfer context. */
    if (data_info.data_type == 0) {
        at_error();
        return;
    }
    if (hdr_id != data_info.data_id) {
        at_error();
        return;
    }

    /* Per-type chunk layout checks. */
    switch (data_info.data_type) {
        case DATA_TYPE_REPORT:
        {
            if (hdr_page != 0 || hdr_chunk != 0) {
                at_error();
                return;
            }
        }
        break;

        case DATA_TYPE_CONFIG:
        {
            if (hdr_page != 0 || hdr_chunk != 0) {
                at_error();
                return;
            }
        }
        break;
        
        case DATA_TYPE_LARGE:
        {
            at_error();
            return;
            // Implement later
        }
        break;

        case DATA_TYPE_OTA:
        {
            at_error();
            return;
            // Implement later
        }

        default:
        {
            at_error();
            return;
        }
        break;
    }

    LOG_INF("AT#DATA id=%u page=%u chunk=%u len=%d",
            hdr_id, hdr_page, hdr_chunk, decoded);

    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#DATA: id=%u page=%u chunk=%u len=%d", hdr_id, hdr_page, hdr_chunk, decoded);
    at_send_line(resp);
    at_ok();
}

/* ---- Dispatch ------------------------------------------------------- */

/* Flat strstr() chain — no dispatch table, matching the h745 codebase. */
static void dispatch(char *line)
{
    /* h745 commands are case-sensitive; we accept exactly that. */
    if (strstr(line, "AT#VERSION?") != NULL) {
        cmd_version();
    } else if (strstr(line, "AT#DEVTYPE?") != NULL) {
        cmd_devtype();
    } else if (strstr(line, "AT#DEVID?") != NULL) {
        cmd_devid();
    } else if (strstr(line, "AT#SN?") != NULL) {
        cmd_sn();
    } else if (strstr(line, "AT#HOP?") != NULL) {
        cmd_hop();
    } else if (strstr(line, "AT#REBOOT") != NULL) {
        cmd_reboot();
    } else if (strstr(line, "AT#DATAINIT") != NULL) {
        char *p = strstr(line, "AT#DATAINIT") + strlen("AT#DATAINIT");
        cmd_data_init(p);
    } else if (strstr(line, "AT#DATA") != NULL) {
        char *p = strstr(line, "AT#DATA") + strlen("AT#DATA");
        cmd_data(p);
    } else if (strstr(line, "AT") != NULL) {
        /* Plain "AT" — must come last because it matches any AT command. */
        /* Make sure no '+' / '#' follows the AT, otherwise it's an unknown
         * command, not a bare AT. */
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
    char line[SLM_UART_AT_COMMAND_LEN];

    /* Non-blocking peek: if no message is ready, return immediately so the
     * outer state machine can do other work. */
    if (!slm_at_wait_for_message(0)) {
        return;
    }

    slm_at_get_message_copy(line);
    dispatch(line);
}
