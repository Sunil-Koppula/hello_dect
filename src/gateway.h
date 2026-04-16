#ifndef GATEWAY_H
#define GATEWAY_H

#include <stdint.h>

/* Initialize gateway module. */
int gateway_init(void);

/* Process a received packet (called from gateway main loop). */
void gateway_process_rx(const uint8_t *data, uint16_t sender_id, int16_t rssi_2);

/* Gateway main loop (RX → process → TX, runs forever). */
void gateway_main(void);

#endif /* GATEWAY_H */
