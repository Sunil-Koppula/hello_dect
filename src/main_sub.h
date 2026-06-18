#ifndef MAIN_SUB_H
#define MAIN_SUB_H

#include <stdint.h>

typedef enum
{
    MAIN_SUB_INIT = 0,
    MAIN_SUB_RX_WINDOW,
    MAIN_SUB_TX_PROCESS,
    MAIN_SUB_ERROR
} main_sub_state_t;

void main_sub_run(void);

#endif /* MAIN_SUB_H */