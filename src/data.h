#ifndef DATA_H
#define DATA_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "protocol.h"
#include "psram.h"
#include "timeout.h"
#include "slm_at_main.h"

#define DATA_PSRAM_BASE			PSRAM_DATA_BASE
#define DATA_SLOT_COUNT			512
#define MAX_REPORT_SIZE			256
#define DATA_PSRAM_SIZE			(DATA_SLOT_COUNT * MAX_REPORT_SIZE)
#define DATA_SLOT_TIMEOUT_MS	(30 * 1000)  /* 30 seconds idle timeout for report slots */
#define PROCESS_REPORT_SLOTS	8  /* limit how many report slots we process per tick to avoid long blocking */
#define DATA_DUP_TIMEOUT_MS		(30 * 1000) /* 30 seconds timeout for duplicate report detection */
#define SENDER_TIMEOUT_MS		(2 * 60 * 1000) /* 2 minutes timeout for sender to wait for Completion */

struct report_sender {
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
	struct nbtimeout timeout;
};

extern struct report_sender sender;

/* Initialize the report module. Must be called after psram_init(). */
int report_init(void);

/* Call from main loop to expire stale slots. */
void report_tick(void);

/* Validate an AT report before processing. */
int validate_at_report(const slm_at_structure_t *report, uint8_t priority, const uint8_t *data);

/* Release a report slot by its ID. */
int report_slot_release_by_id(uint16_t device_id, uint16_t report_id, bool is_success);

/* TX helpers */
int send_report_init(report_init_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority);
int send_report_init_ack(report_init_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id);
int send_report_chunk(report_chunk_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority);
int send_report_chunk_ack(report_chunk_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id);
int send_report_received(report_received_t *pkt, uint16_t dst_id, uint8_t dst_type);
int send_report_received_ack(report_received_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id);

/* RX handlers — wire into each device's RX dispatch switch. */
void handle_report_init(const report_init_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_report_init_ack(const report_init_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_report_chunk(const report_chunk_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_report_chunk_ack(const report_chunk_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_report_received(const report_received_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_report_received_ack(const report_received_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);

#endif /* DATA_H */
