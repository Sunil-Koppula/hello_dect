#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

/* Initialize sensor module. Checks EEPROM for existing pairing. */
int sensor_init(void);

/* Process a received packet (called from main loop RX processing). */
void sensor_process_rx(const uint8_t *data, uint16_t sender_id, int16_t rssi_2);

/* Sensor main loop (RX → process → TX → tick, runs forever). */
void sensor_main(void);

#endif /* SENSOR_H */
