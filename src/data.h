#ifndef DATA_H
#define DATA_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "protocol.h"
#include "psram.h"

#define DATA_PSRAM_BASE          PSRAM_DATA_BASE
#define DATA_SLOT_COUNT          512
#define DATA_MAX_TRANSFER_SIZE   256
#define DATA_PSRAM_SIZE          (DATA_SLOT_COUNT * DATA_MAX_TRANSFER_SIZE)
#define DATA_SLOT_TIMEOUT_MS     30000  /* free slot if idle this long */

struct data_sender {
	bool     active;
	uint16_t dst_id;
	uint16_t gen_device_id;  /* original origin — preserved across hops */
	uint8_t   data_id;       /* ID of the data being sent (for the sender's reference, e.g. to match with ACKs) */
	uint8_t  priority;
	uint16_t total_size;
	uint8_t  chunk_count;
	uint8_t  last_chunk_size;
	uint8_t  next_chunk;
	uint32_t crc32;
};

extern struct data_sender sender;

/* Initialize the data module. Must be called after psram_init(). */
int data_init(void);

/* Call from main loop to expire stale slots. */
void data_tick(void);

/* TX helpers */
int send_data_init(data_init_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority);
int send_data_init_ack(data_init_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id);
int send_data_chunk(data_chunk_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority);
int send_data_chunk_ack(data_chunk_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id);
int send_data_received(data_receive_t *pkt, uint16_t dst_id, uint8_t dst_type);

/* RX handlers — wire into each device's RX dispatch switch. */
void handle_data_init(const data_init_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_data_init_ack(const data_init_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_data_chunk(const data_chunk_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_data_chunk_ack(const data_chunk_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_data_received(const data_receive_t *pkt, uint16_t dst_id, int16_t rssi_2);

#endif /* DATA_H */
