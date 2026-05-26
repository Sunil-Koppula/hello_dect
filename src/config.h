#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "protocol.h"
#include "psram.h"
#include "timeout.h"

#define CONFIG_PSRAM_BASE        PSRAM_CONFIG_BASE
#define CONFIG_SLOT_COUNT        512
#define CONFIG_MAX_SIZE          (128 + 32) /* config data max size (128 bytes) + CRC */
#define CONFIG_PSRAM_SIZE        (CONFIG_SLOT_COUNT * CONFIG_MAX_SIZE)

struct config_slot {
	bool active;
	bool is_sent;
	bool is_ready;
	bool is_applied;
	uint16_t dst_device_id;
	uint8_t dst_device_type;
	uint16_t config_id;
	uint8_t config_len;
	uint32_t config_crc32;
	struct nbtimeout timeout;
};

extern struct config_slot config_slots[CONFIG_SLOT_COUNT];

/* Initialize the config module. Must be called after psram_init(). */
int config_init(void);

/* Call from main loop to expire stale slots. */
void config_tick(void);

/* Validate a config slot and it is used for serial communication */
int validate_config_slot(uint16_t device_id, uint8_t device_type, uint16_t config_id, uint8_t config_len, uint32_t config_crc32);

/* TX helpers */
int send_config(config_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority);
int send_config_ack(config_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id);
int send_config_received(config_received_t *pkt, uint16_t dst_id, uint8_t dst_type);

/* RX handlers — wire into each device's RX dispatch switch. */
void handle_config(const config_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_config_ack(const config_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_config_received(const config_received_t *pkt, uint16_t dst_id, int16_t rssi_2);

#endif /* CONFIG_H */