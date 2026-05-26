/*
+------------------------------------------------------------------------------+
| IRISS Inc.
| -----------------------------------------------------------------------------
| Copyright 2023-2025 (c) IRISS Inc. All rights reserved.
+------------------------------------------------------------------------------+
*/
/**
 *  @file    slm_at_sub.c
 *  @brief   Argument extraction helpers for the SLM AT command processor.
 *           Literal port of h745-zephyr-gateway/src/slm_at_sub.c.
 */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(slm_at_sub, CONFIG_MAIN_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include "slm_at_sub.h"

int slm_at_extract_int(char *str, uint32_t position, int64_t *data)
{
    int      err = 0;
    uint32_t str_size, i, j;
    uint32_t digit_pos[16];
    bool     digit_start_flag = true;
    char     temp_buf[SLM_UART_STRING_MESSAGE_SIZE_MAX + 8];

    *data = 0;
    memset(digit_pos, 0, sizeof(digit_pos));

    str_size = strlen(str);
    if (str_size > SLM_UART_STRING_MESSAGE_SIZE_MAX) {
        return -ENOEXEC;
    }
    strcpy(temp_buf, str);

    for (i = 0, j = 0; i < str_size; i++) {
        if ((isdigit((unsigned char)temp_buf[i]) == false) &&
            (temp_buf[i] != '-')) {
            temp_buf[i]      = 0;
            digit_start_flag = true;
        } else if (digit_start_flag) {
            digit_pos[j] = i;
            if (j < 15) {
                j++;
            }
            digit_start_flag = false;
        }
    }

    if (j > position) {
        *data = atoll(&temp_buf[digit_pos[position & 0x0F]]);
        err   = 0;
    } else {
        err = -ENOEXEC;
    }
    return err;
}

int slm_at_extract_hex_int(char *str, uint32_t position, int64_t *data)
{
    int      err = 0;
    uint32_t str_size, i, j;
    uint32_t digit_pos[16];
    bool     digit_start_flag = true;
    char     temp_buf[SLM_UART_STRING_MESSAGE_SIZE_MAX + 8];

    *data = 0;
    memset(digit_pos, 0, sizeof(digit_pos));

    str_size = strlen(str);
    if (str_size > SLM_UART_STRING_MESSAGE_SIZE_MAX) {
        return -ENOEXEC;
    }
    strcpy(temp_buf, str);

    for (i = 0, j = 0; i < str_size; i++) {
        if ((isxdigit((unsigned char)temp_buf[i]) == false) &&
            (temp_buf[i] != '-')) {
            temp_buf[i]      = 0;
            digit_start_flag = true;
        } else if (digit_start_flag) {
            digit_pos[j] = i;
            if (j < 15) {
                j++;
            }
            digit_start_flag = false;
        }
    }

    if (j > position) {
        *data = strtoll(&temp_buf[digit_pos[position & 0x0F]], NULL, 16);
        err   = 0;
    } else {
        err = -ENOEXEC;
    }
    return err;
}

int slm_at_extract_string(char *str, uint32_t position, char *estr)
{
    int      err = 0;
    uint32_t str_size, i, j;
    uint32_t str_pos[16];
    bool     str_start_flag = true;
    char     temp_buf[SLM_UART_STRING_MESSAGE_SIZE_MAX + 8];

    memset(str_pos, 0, sizeof(str_pos));

    str_size = strlen(str);
    if (str_size > SLM_UART_STRING_MESSAGE_SIZE_MAX) {
        return -ENOEXEC;
    }
    strcpy(temp_buf, str);

    if (position != ALL_POSITIONS) {
        for (i = 0, j = 0; i < str_size; i++) {
            char cmp_byte = temp_buf[i];
            if (cmp_byte == ',' ||
                cmp_byte == '=' ||
                cmp_byte == ':' ||
                cmp_byte == 0x0D ||
                cmp_byte == 0x0A) {
                temp_buf[i]    = 0;
                str_start_flag = true;
            } else if (str_start_flag) {
                str_pos[j] = i;
                if (j < 15) {
                    j++;
                }
                str_start_flag = false;
            }
        }
    }

    strcpy(estr, &temp_buf[str_pos[position & 0x0F]]);
    return err;
}
