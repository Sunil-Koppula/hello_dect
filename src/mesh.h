/*
 * Mesh helpers for DECT NR+ mesh network
 */

#ifndef MESH_H
#define MESH_H

#include <stdint.h>
#include <stddef.h>
#include "product_info.h"
#include "protocol.h"

#define PACKET_TIMEOUT_MS                   500
#define PING_TIMEOUT_MS                     (2 * 60 * 1000)
#define PAIR_RESPONSE_COLLECT_WINDOW_MS     3000
#define PACKET_MAX_RETRIES                  5
#define MAX_COMM_FAILURES                   3
#define MAX_PAIR_RESPONSE_CANDIDATES        16

static uint16_t temp_id;

/* Call each main loop cycle to check response collection timer. */
void mesh_tick(void);

uint64_t mesh_time_get(void);
void mesh_time_set(uint64_t remote_mesh_time);

void set_temp_id(uint16_t id);
uint16_t get_temp_id(void);

uint8_t check_infra_storage(uint16_t device_id, uint8_t device_type, bool is_it_sensor);
bool update_infra_storage(uint16_t device_id, uint8_t hop_num, int16_t rssi_2);
uint8_t check_sensor_storage(uint16_t device_id);
void update_sensor_storage(uint16_t device_id, uint16_t version);
uint8_t check_mesh_storage(uint16_t device_id);
void update_mesh_storage(uint16_t device_id, uint8_t hop_num, uint16_t version, uint16_t connected_device_id, uint8_t sensor_cnt);

#endif /* MESH_H */