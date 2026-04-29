#ifndef DATA_H
#define DATA_H

#include <stdint.h>
#include <stddef.h>
#include "protocol.h"

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
 * Slots live in PSRAM (region 0x100000–0x110000, 64KB) so concurrent
 * transfers from multiple senders don't compete for SRAM. Each slot
 * reserves DATA_MAX_TRANSFER_SIZE (4KB) regardless of actual transfer
 * size — keeps the allocator trivial.
 */

#define DATA_PSRAM_BASE          0x100000
#define DATA_SLOT_COUNT          64
#define DATA_MAX_TRANSFER_SIZE   4096
#define DATA_PSRAM_SIZE          (DATA_SLOT_COUNT * DATA_MAX_TRANSFER_SIZE)
#define DATA_SLOT_TIMEOUT_MS     30000  /* free slot if idle this long */

/* Initialize the data module. Must be called after psram_init(). */
int data_init(void);

/* Call from main loop to expire stale slots. */
void data_tick(void);

/* TX helpers */
int send_data_init(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_init_t *pkt);
int send_data_init_ack(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_init_ack_t *pkt);
int send_data_chunk(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_chunk_t *pkt);
int send_data_chunk_ack(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_chunk_ack_t *pkt);

/* RX handlers — wire into each device's RX dispatch switch. */
void handle_data_init(const data_init_t *pkt, uint16_t sender_id, int16_t rssi_2);
void handle_data_init_ack(const data_init_ack_t *pkt, uint16_t sender_id, int16_t rssi_2);
void handle_data_chunk(const data_chunk_t *pkt, uint16_t sender_id, int16_t rssi_2);
void handle_data_chunk_ack(const data_chunk_ack_t *pkt, uint16_t sender_id, int16_t rssi_2);

/* Call from main loop to expire stale slots. */
void data_tick(void);

#endif /* DATA_H */
