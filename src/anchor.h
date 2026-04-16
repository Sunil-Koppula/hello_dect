#ifndef ANCHOR_H
#define ANCHOR_H

#include <stdint.h>
#include <stdbool.h>
#include "product_info.h"

/* Initialize anchor module. Checks EEPROM for existing pairing. */
int anchor_init(void);

/* Process a received packet. */
void anchor_process_rx(const uint8_t *data, uint16_t sender_id, int16_t rssi_2);

/* Anchor main loop (runs forever). */
void anchor_main(void);

#endif /* ANCHOR_H */
