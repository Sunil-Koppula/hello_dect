/*
+------------------------------------------------------------------------------+
| IRISS Inc.
| -----------------------------------------------------------------------------
| Copyright 2023-2025 (c) IRISS Inc. All rights reserved.
+------------------------------------------------------------------------------+
*/
/**
 *  @file    slm_at_sub.h
 *  @brief   Argument extraction helpers for the SLM AT command processor.
 *           Literal port of h745-zephyr-gateway/src/slm_at_sub.h.
 */
#ifndef SLM_AT_SUB_H_
#define SLM_AT_SUB_H_

#include <stdint.h>
#include <stdbool.h>
#include "slm_at_uart.h"

#define AT_CMD_MAX_SIZE     140
#define AT_OTA_CMD_MAX_SIZE 4096

/* String/Data positions. */
#define FIRST_POSITION  ((uint32_t)0)
#define SECOND_POSITION ((uint32_t)1)
#define THIRD_POSITION  ((uint32_t)2)
#define FOURTH_POSITION ((uint32_t)3)
#define FIFTH_POSITION  ((uint32_t)4)
#define ALL_POSITIONS   ((uint32_t)0xFFFFFFFF)

#define AT_WAIT_HUNDRED_MS   100
#define AT_WAIT_ONE_SEC      1000
#define AT_WAIT_TWO_SEC      (AT_WAIT_ONE_SEC * 2)
#define AT_WAIT_FIVE_SEC     (AT_WAIT_ONE_SEC * 5)
#define AT_WAIT_SEVEN_SEC    (AT_WAIT_ONE_SEC * 7)
#define AT_WAIT_TEN_SEC      (AT_WAIT_ONE_SEC * 10)

#ifdef __cplusplus
extern "C" {
#endif

int slm_at_extract_int(char *str, uint32_t position, int64_t *data);
int slm_at_extract_hex_int(char *str, uint32_t position, int64_t *data);
int slm_at_extract_string(char *str, uint32_t position, char *estr);

#ifdef __cplusplus
}
#endif

#endif /* SLM_AT_SUB_H_ */
