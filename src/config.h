#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "protocol.h"
#include "psram.h"
#include "timeout.h"
#include "slm_at_main.h"

#define CONFIG_SLOT_COUNT			MAX_DEVICES
#define CONFIG_PSRAM_SIZE			(CONFIG_SLOT_COUNT * MAX_CONFIG_SIZE)
#define PROCESS_CONFIG_SLOTS		8 // Max number of config slots to process per tick to avoid long blocking in main loop
#define GATEWAY_CONFIG_TIMEOUT_MS	(2 * 60 * 1000) // 2 minutes timeout for gateway
#define SENSOR_CONFIG_TIMEOUT_MS	(30 * 1000) // 30 seconds timeout for sensor
#define SENSOR_CONFIG_MAX_RETRIES	3 // Max retries for sensor to apply config

struct config_slot {
	bool active;
	bool is_sent;
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

/* Validate an AT config command and its payload. */
int validate_at_config(const slm_at_structure_t *config, const uint8_t *data);

/* Release a config slot by its ID. */
int config_slot_release_by_id(uint16_t config_id, bool is_success);

/* TX helpers */
int send_config(config_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority);
int send_config_ack(config_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id);
int send_config_received(config_received_t *pkt, uint16_t dst_id, uint8_t dst_type);
int send_config_received_ack(config_received_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id);

/* RX handlers — wire into each device's RX dispatch switch. */
void handle_config(const config_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_config_ack(const config_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_config_received(const config_received_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_config_received_ack(const config_received_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);

#endif /* CONFIG_H */