#ifndef LARGE_DATA_H
#define LARGE_DATA_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "protocol.h"
#include "psram.h"
#include "timeout.h"

#define LARGE_DATA_CHUNK_SIZE                   SEND_LARGE_DATA_MAX
#define LARGE_DATA_CHUNKS_PER_PAGE              32
#define LARGE_DATA_PAGE_SIZE                    (LARGE_DATA_CHUNK_SIZE * LARGE_DATA_CHUNKS_PER_PAGE)
#define LARGE_DATA_MAX_PAGE_COUNT               (PSRAM_LARGE_DATA_SIZE / LARGE_DATA_PAGE_SIZE)

#define LARGE_DATA_PSRAM_BASE                   PSRAM_LARGE_DATA_BASE
#define LARGE_DATA_PSRAM_SIZE                   (LARGE_DATA_MAX_PAGE_COUNT * LARGE_DATA_PAGE_SIZE)

#define LARGE_DATA_SENDER_SLOT_COUNT            1
#define LARGE_DATA_RECEIVER_SLOT_COUNT          64
#define LARGE_DATA_MAX_TRANSFER_SIZE            (200 * 1024)    /* some buffer for metadata */

#define LARGE_DATA_SLOT_TIMEOUT_MS              (30 * 1000)     /* free slot if idle this long */
#define PACKET_LARGE_DATA_TIMEOUT_MS            150             /* 150ms timeout for waiting ACKs before retrying */
#define LD_SENDER_TIMEOUT_MS                    (30 * 1000)     /* 30 seconds timeout for sender to wait for transfer completion before giving up */

#define LD_CRC_VERIFY_STAGE_SIZE                1024  /* Read and process data in 1KB stages for CRC verification */

struct large_data_sender {
    bool        active;
    uint16_t    dst_id;
    uint16_t    gen_device_id;  /* original origin — preserved across hops */
    uint8_t     data_id;        /* ID of the data being sent (for the sender's reference, e.g. to match with ACKs) */
    uint8_t     priority;
    uint32_t    total_size;
    uint16_t    page_count;
    uint16_t    last_chunk_size;
    uint16_t    total_chunks;
    uint16_t    next_chunk;
    uint32_t    crc32;
    uint32_t    base_addr;      /* base address in PSRAM where the data to be sent is stored */
    struct nbtimeout timeout;
};

struct large_data_slot {
    bool        active;
    bool        upstream_ready;
    bool        is_sent;
    bool        is_transfered;
    uint16_t    gen_device_id;
    uint8_t     data_id;
    uint8_t     priority;
    uint32_t    total_size;
    uint8_t     page_count;
    uint16_t    last_chunk_size;
    uint16_t    total_chunks;
    uint32_t    crc32;
    uint16_t    received_count;
    uint8_t     bitmap[((LARGE_DATA_MAX_TRANSFER_SIZE / LARGE_DATA_CHUNK_SIZE) + 7) / 8];
    uint32_t    base_addr;
    uint64_t    report_time_ms;
    struct nbtimeout idle_timeout;    
};

extern struct large_data_sender ld_sender[LARGE_DATA_SENDER_SLOT_COUNT];
extern bool is_ld_slot_empty[LARGE_DATA_MAX_PAGE_COUNT];    /* Track empty slots */
extern struct large_data_slot ld_slot[LARGE_DATA_RECEIVER_SLOT_COUNT];

/* Initialize large data module. Must be called after PSRAM initialization */
int large_data_init(void);

/* Call from main loop to expire stale slots */
void large_data_tick(void);

/* AT command handlers */
void cmd_ld_init(const char *args);
void cmd_ld_init_ack(const char *args, bool is_ready);
void cmd_ld_chunk(const char *args);
void cmd_ld_chunk_ack(const char *args, bool is_ready);

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