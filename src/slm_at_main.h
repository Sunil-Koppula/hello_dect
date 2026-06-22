/*
+------------------------------------------------------------------------------+
| IRISS Inc.
| -----------------------------------------------------------------------------
| Copyright 2023-2025 (c) IRISS Inc. All rights reserved.
+------------------------------------------------------------------------------+
*/
/**
 *  @file    slm_at_main.h
 *  @brief   Main entry points for the SLM AT command processor (server side).
 *           Modelled on h745-zephyr-gateway/src/slm_at_main.{c,h} but with the
 *           direction inverted: we receive AT commands and emit responses
 *           instead of sending commands to an external module.
 */
#ifndef SLM_AT_MAIN_H_
#define SLM_AT_MAIN_H_

#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AT_DATA_PAYLOAD_MAX 472

typedef struct {
    uint8_t data_type;
    uint64_t sn;
    uint16_t id;
    int8_t page;
    int8_t chunk;
} at_pending_ack_t;

typedef struct {
    uint16_t device_id;
    uint8_t device_type;
    uint16_t data_id;
    uint16_t data_len;
    uint32_t data_crc32;
} slm_at_structure_t;

/**
 * @brief  Initialize the AT command processor (brings up the UART transport).
 * @return 0 on success, otherwise a negative error code.
 */
int slm_at_init(void);

/**
 * @brief  Drain any queued RX message, parse it, dispatch to a handler,
 *         and emit the response. Call this periodically from each device's
 *         main loop (matches h745's slm_at_run_cycle() pattern).
 */
void slm_at_run_cycle(void);

/**
 * @brief  Extract the idx-th quoted field and parse its contents as a big-endian hex
 *         number into *out. The field must be non-empty and all hex.
 * @param  args  The AT command arguments string.
 * @param  idx   The index of the quoted field to extract.
 * @param  out   Pointer to the output variable.
 * @return 0 on success, otherwise a negative error code.
 */
int field_hex_u64(const char *args, unsigned idx, uint64_t *out);

/**
 * @brief  Decode a hex string into binary data.
 * @param  hex      The hex string.
 * @param  hex_len  The length of the hex string.
 * @param  out      Pointer to the output buffer.
 * @param  out_max  Maximum number of bytes that can be written to out.
 * @return Number of bytes written to out on success, otherwise a negative error code.
 */
int hex_decode(const char *hex, size_t hex_len, uint8_t *out, size_t out_max);

/**
 * @brief  Locate the idx-th double-quoted field in 'args'. Returns a pointer to
 *         the first char inside the quotes and writes its length to *len_out,
 *         or NULL if there is no idx-th complete "..." field.
 */
const char *field_get(const char *args, unsigned idx, size_t *len_out);

/** @brief Emit the standard "\r\nOK\r\n" AT response. */
void at_ok(void);

/** @brief Emit the standard "\r\nERROR\r\n" AT response. */
void at_error(void);

/** @brief Emit "\r\n<s>\r\n" as an AT response line. */
void at_send_line(const char *s);

/** @brief  Set the pending AT acknowledgment. */
void set_at_pending_ack(at_pending_ack_t ack);

#ifdef __cplusplus
}
#endif

#endif /* SLM_AT_MAIN_H_ */
