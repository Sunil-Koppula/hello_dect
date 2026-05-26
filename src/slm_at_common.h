/*
+------------------------------------------------------------------------------+
| IRISS Inc.
| -----------------------------------------------------------------------------
| Copyright 2023-2025 (c) IRISS Inc. All rights reserved.
+------------------------------------------------------------------------------+
*/
/**
 *  @file    slm_at_common.h
 *  @brief   Common definitions for the SLM AT command processor
 *           (literal port of the h745-zephyr-gateway design).
 */
#ifndef _SLM_AT_COMMON_H_
#define _SLM_AT_COMMON_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define SLM_UART_STRING_MESSAGE_SIZE_MAX ((size_t)256)
#define SLM_UART_MIN_10_MS_DELAY_TIME    10
#define SLM_UART_AT_COMMAND_LEN          512

typedef enum {
    SLM_AT_SEND_INIT = 0,
    SLM_AT_SEND_CMD,
    SLM_AT_WAIT_RESP
} slm_at_sm_state_t;

typedef struct {
    size_t  size;
    int8_t  buf[SLM_UART_AT_COMMAND_LEN];
    bool    message_ready;
} data_msg_t;

#endif /* _SLM_AT_COMMON_H_ */
