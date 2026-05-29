#ifndef LARGE_DATA_H
#define LARGE_DATA_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "protocol.h"
#include "psram.h"

#define LARGE_DATA_SLOT_SIZE                    (20 * SEND_DATA_MAX)
#define LARGE_DATA_SLOT_COUNT                   (PSRAM_LARGE_DATA_SIZE / LARGE_DATA_SLOT_SIZE)
#define LARGE_DATA_PSRAM_SIZE                   (LARGE_DATA_SLOT_COUNT * LARGE_DATA_SLOT_SIZE)

#define LARGE_DATA_PSRAM_BASE                   PSRAM_LARGE_DATA_BASE
#define LARGE_DATA_RECEIVER_SLOT_COUNT          64
#define LARGE_DATA_MAX_TRANSFER_SIZE            (200 * 1024)  /* 200KB max transfer size */
#define LARGE_DATA_SLOT_TIMEOUT_MS              60000  /* free slot if idle this long */

extern bool is_ld_slot_empty[LARGE_DATA_SLOT_COUNT];    /* Track empty slots */

struct large_data_sender {
    bool        active;
    uint16_t    dst_id;
    uint16_t    gen_device_id;  /* original origin — preserved across hops */
    uint8_t     data_id;        /* ID of the data being sent (for the sender's reference, e.g. to match with ACKs) */
    uint8_t     priority;
    uint32_t    total_size;
    uint16_t    page_count;
    uint16_t    last_page_size;
    uint16_t    total_chunks;
    uint16_t    next_chunk;
    uint32_t    crc32;
    uint32_t    base_addr;      /* base address in PSRAM where the data to be sent is stored */
};

extern struct large_data_sender ld_sender;

/* Initialize large data module. Must be called after PSRAM initialization */
int large_data_init(void);

/* Call from main loop to expire stale slots */
void large_data_tick(void);

/* TX helpers */
int send_large_data_init(large_data_init_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority);
int send_large_data_init_ack(large_data_init_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id);
int send_large_data_chunk(large_data_chunk_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority);
int send_large_data_chunk_ack(large_data_chunk_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id);
int send_large_data_received(large_data_received_t *pkt, uint16_t dst_id, uint8_t dst_type);
int send_large_data_received_ack(large_data_received_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id);

/* RX handlers — wire into each device's RX dispatch switch. */
void handle_large_data_init(const large_data_init_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_large_data_init_ack(const large_data_init_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_large_data_chunk(const large_data_chunk_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_large_data_chunk_ack(const large_data_chunk_ack_t *pkt, uint16_t dst_id, int16_t rssi_2);
void handle_large_data_received(const large_data_received_t *pkt, uint16_t dst_id, int16_t rssi_2);

#endif /* LARGE_DATA_H */