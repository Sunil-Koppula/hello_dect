#ifndef LARGE_DATA_H
#define LARGE_DATA_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "protocol.h"
#include "psram.h"
#include "timeout.h"

#define LARGE_DATA_CHUNKS_PER_SIZE              32
#define LARGE_DATA_SLOT_SIZE                    (LARGE_DATA_CHUNKS_PER_SIZE * SEND_LARGE_DATA_MAX)
#define LARGE_DATA_SLOT_COUNT                   (PSRAM_LARGE_DATA_SIZE / LARGE_DATA_SLOT_SIZE)
#define LARGE_DATA_PSRAM_SIZE                   (LARGE_DATA_SLOT_COUNT * LARGE_DATA_SLOT_SIZE)

#define LARGE_DATA_PSRAM_BASE                   PSRAM_LARGE_DATA_BASE
#define LARGE_DATA_RECEIVER_SLOT_COUNT          16
#define SOUND_RECORD_DATA_MAX_SIZE              (200 * 1024)  /* 200KB max sound record size */
#define LARGE_DATA_MAX_TRANSFER_SIZE            (SOUND_RECORD_DATA_MAX_SIZE + 64) /* some buffer for metadata */
#define LARGE_DATA_SLOT_TIMEOUT_MS              (30 * 1000)  /* free slot if idle this long */
#define PACKET_LARGE_DATA_TIMEOUT_MS            (100) /* 100ms timeout for waiting ACKs before retrying */
#define LD_SENDER_TIMEOUT_MS                    (3 * 60 * 1000) /* 3 minutes timeout for sender to wait for transfer completion before giving up */

#define LD_CRC_VERIFY_STAGE_SIZE                1024  /* Read and process data in 1KB stages for CRC verification */
#define LD_MAX_SENDER_PROCESS                   1     /* Max concurrent sending transfers tracked */
#define LD_MAX_RECEIVER_PROCESS                 LARGE_DATA_RECEIVER_SLOT_COUNT /* Max concurrent receiving transfers tracked, aligned with receiver slot count */

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
    uint32_t    base_addr;
    struct nbtimeout idle_timeout;    
};

extern struct large_data_sender ld_sender[LD_MAX_SENDER_PROCESS];
extern bool is_ld_slot_empty[LARGE_DATA_SLOT_COUNT];    /* Track empty slots */
extern struct large_data_slot ld_slot[LARGE_DATA_RECEIVER_SLOT_COUNT];

/* Initialize large data module. Must be called after PSRAM initialization */
int large_data_init(void);

/* Call from main loop to expire stale slots */
void large_data_tick(void);

void cmd_ld_init(const char *args);

void cmd_ld_chunk(const char *args);

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