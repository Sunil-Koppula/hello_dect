#ifndef DATA_H
#define DATA_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "protocol.h"
#include "psram.h"

/*
 * Bulk data transfer: chunked unicast with per-chunk ACK + CRC32.
 *
 * Sender flow:
 *   1. data_send() → send DATA_INIT (total_size, chunk_count, crc32)
 *   2. on DATA_INIT_ACK → send DATA_CHUNK[0]
 *   3. on DATA_CHUNK_ACK[i] → send DATA_CHUNK[i+1]
 *   4. on DATA_CHUNK_ACK[last] → transfer done
 *
 * Receiver flow:
 *   1. on DATA_INIT → allocate slot keyed by (gen_device_id, priority),
 *                     ACK with DATA_INIT_ACK
 *   2. on DATA_CHUNK[i] → write chunk to PSRAM slot, ACK, mark received
 *   3. when all chunks received → CRC32 check, log result, free slot
 *
 * Large-transfer slots live in PSRAM partition P3 (6 MB at 0x200000).
 * Each slot reserves DATA_MAX_TRANSFER_SIZE (4 KB) regardless of actual
 * transfer size — keeps the allocator trivial. Concurrent transfers from
 * multiple senders share the partition, not SRAM.
 *
 * Small-transfer (≤256 B) support will land in partition P2 later — kept
 * separate so small transfers don't waste a 4 KB slot each.
 */

#define DATA_PSRAM_BASE          PSRAM_P3_BASE
#define DATA_SLOT_COUNT          64
#define DATA_MAX_TRANSFER_SIZE   4096
#define DATA_PSRAM_SIZE          (DATA_SLOT_COUNT * DATA_MAX_TRANSFER_SIZE)
#define DATA_SLOT_TIMEOUT_MS     30000  /* free slot if idle this long */

/* Single in-flight sender state (test scaffolding — exposed so callers
 * staging an outbound transfer can read gen_device_id, etc.). */
struct data_sender {
	bool     active;
	uint16_t dst_id;
	uint16_t gen_device_id;  /* original origin — preserved across hops */
	uint8_t  priority;
	const uint8_t *buf;
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
int send_data_init(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_init_t *pkt);
int send_data_init_ack(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_init_ack_t *pkt);
int send_data_chunk(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_chunk_t *pkt);
int send_data_chunk_ack(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_chunk_ack_t *pkt);

/* Test entry point: stage `size` bytes of counter-pattern dummy data in
 * PSRAM slot 0, populate sender state, and send DATA_INIT to dst_id.
 * Returns 0 on success, -EBUSY if a transfer is already in flight,
 * -EINVAL on bad size, or a negative errno from PSRAM. */
int data_send_test(uint16_t dst_id, uint16_t size);

/* RX handlers — wire into each device's RX dispatch switch. */
void handle_data_init(const data_init_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_data_init_ack(const data_init_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_data_chunk(const data_chunk_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_data_chunk_ack(const data_chunk_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);

/* Call from main loop to expire stale slots. */
void data_tick(void);

#endif /* DATA_H */
