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

#define AT_DATA_PAYLOAD_MAX 450

/* ---- Public response strings (extern'd from other modules if needed). ---- */
const char AT_RESP_OK[]    = {"\r\nOK\r\n"};
const char AT_RESP_ERROR[] = {"\r\nERROR\r\n"};

typedef struct {
    uint8_t data_type;
    uint64_t sn;
    uint16_t id;
    uint8_t page;
    uint8_t chunk;
} at_pending_ack_t;

static at_pending_ack_t pending_ack = {
    .data_type = 0,
    .sn = 0,
    .id = 0,
    .page = 0,
    .chunk = 0,
};

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

/* AT#CONFIG="<SN>","<ID>","<LEN>","<CRC32>","<DATA>"  (gateway only)
 *
 * One-shot CONFIG push. All fields are uppercase hex strings:
 *   SN    : 8-byte destination serial number
 *   ID    : 2-byte data/config id
 *   LEN   : 2-byte payload length (≤ MAX_CONFIG_SIZE = 128)
 *   CRC32 : crc32_ieee of the binary payload
 *   DATA  : payload hex, 2*LEN chars
 *
 * Validates SN against the mesh table, allocates a config slot, verifies
 * CRC, persists to PSRAM, and marks the slot ready. config_tick() then
 * drives the actual radio TX.
 *
 * On success: '#CONFIG: sn=… id=… len=… crc=0x… slot=…' then OK.
 * On any failure: ERROR.
 */
static void cmd_config(const char *args)
{
    if (get_device_type() != DEVICE_TYPE_GATEWAY) {
        at_error();
        return;
    }

    /* Fields 0..3: sn, id, len, crc32 (all quoted hex). */
    uint64_t v_sn, v_id, v_len, v_crc;
    if (field_hex_u64(args, 0, &v_sn)  != 0) { at_error(); return; }
    if (field_hex_u64(args, 1, &v_id)  != 0) { at_error(); return; }
    if (field_hex_u64(args, 2, &v_len) != 0) { at_error(); return; }
    if (field_hex_u64(args, 3, &v_crc) != 0) { at_error(); return; }

    if (v_id > 0xFFFF || v_len == 0 || v_len > MAX_CONFIG_SIZE || v_crc > 0xFFFFFFFF) {
        at_error();
        return;
    }

    slm_at_structure_t config = {
        .data_id = (uint16_t)v_id,
        .data_len = (uint16_t)v_len,
        .data_crc32 = (uint32_t)v_crc,
    };

    uint64_t sn = v_sn;

    /* Field 4: the payload hex. */
    size_t      hex_len;
    const char *payload_hex = field_get(args, 4, &hex_len);
    if (payload_hex == NULL || hex_len != (size_t)config.data_len * 2) {
        at_error();
        return;
    }

    uint8_t payload[MAX_CONFIG_SIZE];
    int     decoded = hex_decode(payload_hex, hex_len, payload, sizeof(payload));
    if (decoded < 0 || decoded != config.data_len) {
        at_error();
        return;
    }

    /* CRC32 over the payload must match. */
    uint32_t calc_crc32 = crc32_ieee(payload, (size_t)decoded);
    if (calc_crc32 != config.data_crc32) {
        LOG_WRN("AT#CONFIG sn=0x%016llx id=%u CRC mismatch: got 0x%08x, calc 0x%08x",
                (unsigned long long)sn, config.data_id, config.data_crc32, calc_crc32);
        at_error();
        return;
    }

    /* SN must be a known mesh device. */
    uint16_t mesh_idx;
    uint8_t hop_num;
    bool     found = false;
    for (mesh_idx = 0; mesh_idx < mesh_count; mesh_idx++) {
        if (mesh_devices[mesh_idx].serial_num == sn) {
            found = true;
            break;
        }
    }
    if (found) {
        hop_num = get_hop_num(mesh_devices[mesh_idx].device_id, mesh_devices[mesh_idx].device_type);
        if (hop_num == 0xFF) {
            found = false;
            LOG_WRN("AT#CONFIG: SN 0x%016llx in mesh but no route found", (unsigned long long)sn);
        }
    }
    if (!found) {
        LOG_WRN("AT#CONFIG: SN 0x%016llx not in mesh", (unsigned long long)sn);
        at_error();
        return;
    }

    config.device_id = mesh_devices[mesh_idx].device_id;
    config.device_type = mesh_devices[mesh_idx].device_type;

    /* Allocate / reuse a slot, persist to PSRAM. */
    int err = validate_at_config(&config, payload);
    if (err < 0) {
        at_error();
        return;
    }

    LOG_INF("AT#CONFIG accepted: sn=0x%016llx %s ID:%d cfg_id=%u len=%u",
            (unsigned long long)sn,
            device_type_str(config.device_type),
            config.device_id, config.data_id, config.data_len);

    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp),
        "#CONFIG:\"0x%016llx\",\"%u\",\"%u\",\"0x%08lx\"",
        (unsigned long long)sn, config.data_id, config.data_len,
        (unsigned long)config.data_crc32);
    at_send_line(resp);
    at_ok();
}

static void cmd_config_ack(const char *args)
{
    uint64_t v_sn, v_id;
    if (field_hex_u64(args, 0, &v_sn) != 0) {
        LOG_WRN("AT#CONFIG_ACK missing or invalid SN field");
        return;
    }
    if (field_hex_u64(args, 1, &v_id) != 0) {
        LOG_WRN("AT#CONFIG_ACK missing or invalid ID field");
        return;
    }

    if (v_id > 0xFFFF) {
        LOG_WRN("AT#CONFIG_ACK invalid ID value %lu", (unsigned long)v_id);
        return;
    }

    /* The SN field in the downstream ack must match THIS sensor's SN —
     * that's the SN we put in the outgoing AT#CONFIG. Mismatched acks
     * mean the downstream is misbehaving or a stale message slipped in;
     * drop without touching pending_ack so we don't free the wrong slot. */
    if (v_sn != get_serial_number()) {
        LOG_WRN("AT#CONFIG_ACK SN mismatch: got 0x%016llx, expected 0x%016llx", (unsigned long long)v_sn, (unsigned long long)get_serial_number());
        return;
    }

    pending_ack.data_type = DATA_TYPE_CONFIG;
    pending_ack.sn = v_sn;
    pending_ack.id = (uint16_t)v_id;
    pending_ack.page = 0;
    pending_ack.chunk = 0;

    LOG_INF("Received AT#CONFIG_ACK for SN 0x%016llx ID %u",
            (unsigned long long)v_sn, (unsigned)v_id);
}

static void cmd_report(const char *args)
{
    if (get_device_type() != DEVICE_TYPE_SENSOR) {
        at_error();
        return;
    }

    if (get_connected_device_id() == 0xFFFF) {
        LOG_WRN("AT#REPORT received but no paired device");
        at_error();
        return;
    }

    /* Fields 0..4: sn, id, len, prio, crc32 (all quoted hex). */
    uint64_t v_sn, v_id, v_len, v_prio, v_crc;
    if (field_hex_u64(args, 0, &v_sn)  != 0) { at_error(); return; }
    if (field_hex_u64(args, 1, &v_id)  != 0) { at_error(); return; }
    if (field_hex_u64(args, 2, &v_len) != 0) { at_error(); return; }
    if (field_hex_u64(args, 3, &v_prio) != 0) { at_error(); return; }
    if (field_hex_u64(args, 4, &v_crc) != 0) { at_error(); return; }

    if (v_id > 0xFFFF || v_len == 0 || v_len > MAX_REPORT_SIZE || v_crc > 0xFFFFFFFF) {
        at_error();
        return;
    }

    slm_at_structure_t report = {
        .data_id = (uint16_t)v_id,
        .data_len = (uint16_t)v_len,
        .data_crc32 = (uint32_t)v_crc,
    };

    uint64_t sn = v_sn;

    /* Field 5: the payload hex. */
    size_t      hex_len;
    const char *payload_hex = field_get(args, 5, &hex_len);
    if (payload_hex == NULL || hex_len != (size_t)report.data_len * 2) {
        at_error();
        return;
    }

    uint8_t payload[MAX_REPORT_SIZE];
    int     decoded = hex_decode(payload_hex, hex_len, payload, sizeof(payload));
    if (decoded < 0 || decoded != report.data_len) {
        at_error();
        return;
    }

    /* CRC32 over the payload must match. */
    uint32_t calc_crc32 = crc32_ieee(payload, (size_t)decoded);
    if (calc_crc32 != report.data_crc32) {
        LOG_WRN("AT#REPORT sn=0x%016llx id=%u CRC mismatch: got 0x%08x, calc 0x%08x",
                (unsigned long long)sn, report.data_id, report.data_crc32, calc_crc32);
        at_error();
        return;
    }

    /* SN should match this sensor's SN. */
    if (sn != get_serial_number()) {
        LOG_WRN("AT#REPORT SN mismatch: got 0x%016llx, expected 0x%016llx", (unsigned long long)sn, (unsigned long long)get_serial_number());
        at_error();
        return;
    }

    int err = validate_at_report(&report, (uint8_t)v_prio, payload);
    if (err < 0) {
        at_error();
        return;
    }

    LOG_INF("AT#REPORT accepted: sn=0x%016llx %s ID:%d len=%u",
            (unsigned long long)sn,
            device_type_str(get_device_type()),
            report.data_id, report.data_len);
         
    char resp[SLM_UART_STRING_MESSAGE_SIZE_MAX];
    snprintf(resp, sizeof(resp),
        "#REPORT:\"0x%016llx\",\"%u\",\"%u\",\"0x%08lx\"",
        (unsigned long long)sn, report.data_id, report.data_len,
        (unsigned long)report.data_crc32);
    at_send_line(resp);
    at_ok();
}

static void cmd_report_ack(const char *args)
{
    uint64_t v_sn, v_id;
    if (field_hex_u64(args, 0, &v_sn) != 0) {
        LOG_WRN("AT#REPORT_ACK missing or invalid SN field");
        return;
    }
    if (field_hex_u64(args, 1, &v_id) != 0) {
        LOG_WRN("AT#REPORT_ACK missing or invalid ID field");
        return;
    }

    if (v_id > 0xFFFF) {
        LOG_WRN("AT#REPORT_ACK invalid ID value %lu", (unsigned long)v_id);
        return;
    }

    /* SN must be a known mesh device. */
    uint16_t mesh_idx;
    uint8_t hop_num;
    bool     found = false;
    for (mesh_idx = 0; mesh_idx < mesh_count; mesh_idx++) {
        if (mesh_devices[mesh_idx].serial_num == v_sn) {
            found = true;
            break;
        }
    }
    if (found) {
        hop_num = get_hop_num(mesh_devices[mesh_idx].device_id, mesh_devices[mesh_idx].device_type);
        if (hop_num == 0xFF) {
            found = false;
            LOG_WRN("AT#REPORT: SN 0x%016llx in mesh but no route found", (unsigned long long)v_sn);
        }
    }
    if (!found) {
        LOG_WRN("AT#REPORT: SN 0x%016llx not in mesh", (unsigned long long)v_sn);
        return;
    }

    pending_ack.data_type = DATA_TYPE_REPORT;
    pending_ack.sn = v_sn;
    pending_ack.id = (uint16_t)v_id;
    pending_ack.page = 0;
    pending_ack.chunk = 0;

    LOG_INF("Received AT#REPORT_ACK for SN 0x%016llx ID %u",
            (unsigned long long)v_sn, (unsigned)v_id);
}

/* ---- Dispatch ------------------------------------------------------- */

/* Flat strstr() chain — no dispatch table, matching the h745 codebase. */
static void dispatch(char *line)
{
    if (strcmp(line, "OK") == 0) {
        switch (pending_ack.data_type) {
            case DATA_TYPE_CONFIG:
                (void)config_slot_release_by_id(pending_ack.id, true);
                break;

            case DATA_TYPE_REPORT:
                (void)report_slot_release_by_id(pending_ack.id, true);
                break;

            default:
                LOG_WRN("Received OK for unknown pending ack type %u", pending_ack.data_type);
                break;
        }
        /* Consume the pending ack so a later spurious OK doesn't re-fire. */
        pending_ack.data_type = 0;
        return;
    }
    if (strcmp(line, "ERROR") == 0) {
        switch (pending_ack.data_type) {
            case DATA_TYPE_CONFIG:
                (void)config_slot_release_by_id(pending_ack.id, false);
                break;

            case DATA_TYPE_REPORT:
                (void)report_slot_release_by_id(pending_ack.id, false);
                break;

            default:
                LOG_WRN("Received ERROR for unknown pending ack type %u", pending_ack.data_type);
                break;
        }
        pending_ack.data_type = 0;
        return;
    }

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        {
            if (strstr(line, "AT#CONFIG") != NULL) {
                char *p = strstr(line, "AT#CONFIG") + strlen("AT#CONFIG");
                cmd_config(p);
                return;
            } else if (strstr(line, "AT#OTA") != NULL) {
                // Implement later
                at_ok();
                return;
            } else if (strstr(line, "#REPORT") != NULL) {
                char *p = strstr(line, "#REPORT");
                cmd_report_ack(p);
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
            } else if (strstr(line, "AT#LD") != NULL) {
                // Implement later
                at_ok();
                return;
            } else if (strstr(line, "#CONFIG") != NULL) {
                char *p = strstr(line, "#CONFIG");
                cmd_config_ack(p);
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
