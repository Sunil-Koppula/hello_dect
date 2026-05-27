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

typedef struct {
    uint8_t data_type;
    uint16_t data_id;
    int  data_idx;
    uint8_t page;
    uint8_t chunk;
} data_info_t;

static data_info_t data_info;

#define AT_DATA_PAYLOAD_MAX 450

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

/* Locate the idx-th double-quoted field in 'args'. On success returns a
 * pointer to the first char inside the quotes and writes its length (chars
 * between the quotes, may be 0) to *len_out. Returns NULL if there is no
 * idx-th complete "..." field. */
static const char *field_get(const char *args, unsigned idx, size_t *len_out)
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
static int field_hex_u64(const char *args, unsigned idx, uint64_t *out)
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

/* AT#DATAINIT="<SN>","<TYPE>","<ID>","<LEN>","<CRC>"  — all fields hex. */
static int parse_data_init(const char *args, data_init_info_t *out)
{
    if (args == NULL || out == NULL) {
        return -EINVAL;
    }

    uint64_t sn, dtype, did, dlen, crc;
    if (field_hex_u64(args, 0, &sn)    != 0) return -EINVAL;
    if (field_hex_u64(args, 1, &dtype) != 0) return -EINVAL;
    if (field_hex_u64(args, 2, &did)   != 0) return -EINVAL;
    if (field_hex_u64(args, 3, &dlen)  != 0) return -EINVAL;
    if (field_hex_u64(args, 4, &crc)   != 0) return -EINVAL;

    if (dtype > 0xFF || did > 0xFFFF || dlen > 0xFFFF || crc > 0xFFFFFFFF) {
        return -EINVAL;
    }

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
        {
            return 0;
        }
        break;

        case DATA_TYPE_CONFIG:
        {
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
        break;

        case DATA_TYPE_LARGE:
        {
            return 0;
        }
        break;

        case DATA_TYPE_OTA:
        {
            return 0;
        }
        break;

        default:
        {
            return -EINVAL;
        }
        break;
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

static void cmd_get_version(void)
{
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#VERSION: %d", FIRMWARE_VERSION);
    at_send_line(resp);
}

static void cmd_get_devtype(void)
{
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#DEVTYPE: %s", device_type_name());
    at_send_line(resp);
}

static void cmd_get_devid(void)
{
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#DEVID: %u", radio_get_device_id());
    at_send_line(resp);
}

static void cmd_get_sn(void)
{
    char     resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    uint64_t sn = SERIAL_NUMBER;
    snprintf(resp, sizeof(resp), "#SN: 0x%08x%08x",
             (unsigned)(sn >> 32), (unsigned)(sn & 0xFFFFFFFF));
    at_send_line(resp);
}

static void cmd_get_hop(void)
{
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#HOP: %u", DEVICE_HOP_NUMBER);
    at_send_line(resp);
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

    SERIAL_NUMBER = sn;
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp), "#SN: 0x%08x%08x",
             (unsigned)(sn >> 32), (unsigned)(sn & 0xFFFFFFFF));
    at_send_line(resp);
}

static void cmd_reboot(void)
{
    at_ok();
    k_msleep(50);
    sys_reboot(SYS_REBOOT_COLD);
}

static void cmd_get_data(void)
{
    if (data_info.data_type == 0) {
        char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
        snprintf(resp, sizeof(resp), "#DATAINFO: ""%u""", 0);
        at_send_line(resp);
    } else {
        char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
        snprintf(resp, sizeof(resp), "#DATAINFO: ""%u"",""%u"",""%d"",""%u"",""%u""",
                 data_info.data_type, data_info.data_id, data_info.data_idx, data_info.page, data_info.chunk);
        at_send_line(resp);
    }
}

/* AT#DATAINIT="<SN>","<TYPE>","<ID>","<LEN>","<CRC>" */
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
}

/* AT#DATA="<ID>","<PAGE>","<CHUNK>","<CRC>","<DATA>"
 *
 * All fields are quoted hex strings:
 *   ID    : data id (must match the most recent AT#DATAINIT)
 *   PAGE  : page number
 *   CHUNK : chunk number within the page
 *   CRC   : CRC32-IEEE of the payload bytes
 *   DATA  : payload, hex-encoded, ≤ AT_DATA_PAYLOAD_MAX (450) bytes
 *
 * Radio still emits SEND_DATA_MAX (180 B) per radio chunk; the firmware
 * is expected to split each AT chunk into 1..N radio chunks when forwarding.
 *
 * Chunking rules by data_type (set via AT#DATAINIT):
 *   CONFIG : exactly one chunk  → page=0, chunk=0
 *   REPORT : exactly one chunk  → page=0, chunk=0
 *   LARGE  : pages × 20 chunks  → page=0..N-1, chunk=0..19
 *
 * A CRC32 mismatch on the chunk payload returns ERROR so the host can retry
 * just that chunk.
 */
static void cmd_data(const char *args)
{
    /* Fields 0..3: id, page, chunk, crc (all quoted hex). */
    uint64_t v_id, v_page, v_chunk, v_crc;
    if (field_hex_u64(args, 0, &v_id)    != 0) { at_error(); return; }
    if (field_hex_u64(args, 1, &v_page)  != 0) { at_error(); return; }
    if (field_hex_u64(args, 2, &v_chunk) != 0) { at_error(); return; }
    if (field_hex_u64(args, 3, &v_crc)   != 0) { at_error(); return; }

    if (v_id > 0xFFFF || v_page > 0xFF || v_chunk > 0xFF || v_crc > 0xFFFFFFFF) {
        at_error();
        return;
    }
    uint16_t hdr_id    = (uint16_t)v_id;
    uint8_t  hdr_page  = (uint8_t)v_page;
    uint8_t  hdr_chunk = (uint8_t)v_chunk;
    uint32_t hdr_crc32 = (uint32_t)v_crc;

    /* Field 4: the payload hex. */
    size_t      payload_chars;
    const char *payload_hex = field_get(args, 4, &payload_chars);
    if (payload_hex == NULL || payload_chars == 0 || (payload_chars % 2) != 0) {
        at_error();
        return;
    }
    if ((payload_chars / 2) > AT_DATA_PAYLOAD_MAX) {
        at_error();
        return;
    }

    uint8_t payload[AT_DATA_PAYLOAD_MAX];
    int     decoded = hex_decode(payload_hex, payload_chars,
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

            if(validate_config_data(data_info.data_idx, data_info.data_id, payload) != 0) {
                at_error();
                return;
            }

            // Reset the data_info to be ready for the next config transfer
            data_info.data_type = 0;
            data_info.data_id = 0;
            data_info.data_idx = -1;
            data_info.page = 0;
            data_info.chunk = 0;
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
}

/* ---- Dispatch ------------------------------------------------------- */

/* Flat strstr() chain — no dispatch table, matching the h745 codebase. */
static void dispatch(char *line)
{
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
    } else if (strstr(line, "AT#DATA?") != NULL) {
        cmd_get_data();
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
    data_info.data_type = 0;
    data_info.data_id = 0;
    data_info.data_idx = -1;
    data_info.page = 0;
    data_info.chunk = 0;

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
