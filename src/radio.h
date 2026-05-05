#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include <stddef.h>
#include <zephyr/kernel.h>
#include <nrf_modem_dect_phy.h>

/* Semaphores for synchronizing modem operations. */
extern struct k_sem operation_sem;
extern struct k_sem deinit_sem;

/* Flag indicating a fatal error occurred in a callback. */
extern bool radio_exit;

/* PHY event handler */
void dect_phy_event_handler(const struct nrf_modem_dect_phy_event *evt);

/* PHY config parameters */
extern struct nrf_modem_dect_phy_config_params dect_phy_config_params;

/* Set/get the 16-bit short device ID used in TX headers. */
void radio_set_device_id(uint16_t id);
uint16_t radio_get_device_id(void);

/* Transmit and receive functions.
 * packet_length: number of subslots (1–15). Use 0 for auto-calc from data_len. */
int transmit(uint32_t handle, void *data, size_t data_len, uint8_t packet_length);
int receive(uint32_t handle, uint32_t duration_ms);

#endif /* RADIO_H */
