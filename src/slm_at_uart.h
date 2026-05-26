/*
+------------------------------------------------------------------------------+
| IRISS Inc.
| -----------------------------------------------------------------------------
| Copyright 2023-2025 (c) IRISS Inc. All rights reserved.
+------------------------------------------------------------------------------+
*/
/**
 *  @file    slm_at_uart.h
 *  @brief   Async UART transport for the SLM AT command processor.
 *           Literal port of h745-zephyr-gateway/src/slm_at_uart.{c,h}.
 */
#ifndef SLM_AT_UART_H_
#define SLM_AT_UART_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "slm_at_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the AT UART (async API, RX/TX ring buffers, workqueues).
 * @return 0 on success, otherwise a negative error code.
 */
int slm_at_uart_init(void);

/**
 * @brief Write data to the TX ring buffer and trigger sending.
 */
int slm_at_tx_write(const uint8_t *data, size_t len, bool print_full_debug);

/**
 * @brief Block (cooperatively) until an AT message has been assembled.
 * @return true if a message is ready within w_time ms, false on timeout.
 */
bool slm_at_wait_for_message(uint32_t w_time);

/**
 * @brief Copy the latest assembled AT message into the caller's buffer
 *        and clear the message_ready flag.
 */
void slm_at_get_message_copy(char *pcopy_str);

#ifdef __cplusplus
}
#endif

#endif /* SLM_AT_UART_H_ */
