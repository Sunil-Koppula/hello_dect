#ifndef MAIN_SUB_H
#define MAIN_SUB_H

#include <stdint.h>

typedef enum
{
    MAIN_SUB_INIT = 0,
    MAIN_SUB_RX_WINDOW,
    MAIN_SUB_RX_PROCESS,
    MAIN_SUB_TX_PROCESS,
    MAIN_SUB_TRACKER,
    MAIN_SUB_ERROR
} main_sub_state_t;

#define PAIR_TIMEOUT_MS     500
#define PAIR_MAX_RETRIES    5

/* Gateway main loop (RX → process → TX, runs forever). */
void gateway_main(void);

/* Anchor main loop (RX → process → TX, runs forever). */
void anchor_main(void);

/* Sensor main loop. */
void sensor_main(void);

#endif /* MAIN_SUB_H */