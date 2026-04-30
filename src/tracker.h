#ifndef TRACKER_H
#define TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include "timeout.h"
#include "product_info.h"
#include "queue.h"
#include "psram.h"

#define TRACKER_MAX_ENTRIES (MAX_SENSORS + MAX_ANCHORS)
#define TRACKER_PAYLOAD_MAX QUEUE_DATA_MAX

/* Tracker payload storage lives at the start of PSRAM partition P1.
 * Layout: TRACKER_MAX_ENTRIES × TRACKER_PAYLOAD_MAX bytes
 *   TRACKER_MAX_ENTRIES = MAX_SENSORS + MAX_ANCHORS = 136
 *   TRACKER_PAYLOAD_MAX = QUEUE_DATA_MAX = 210 bytes
 *   Used: 136 × 210 = 28,560 bytes (~28KB)
 *   Allocated: 32KB (0x8000) — rest of P1 reserved for future use
 */
#define TRACKER_PSRAM_BASE   PSRAM_P1_BASE
#define TRACKER_PSRAM_SIZE   0x8000  /* 32KB */

/* Tracked device entry. */
struct data_tracker {
	uint16_t dst_id;
	uint16_t prev_id;
	uint8_t tracking_id;
	uint8_t packet_type;    /* packet_type_t — what was sent */
	struct nbtimeout timeout;
	bool active;
	uint16_t payload_len;   /* 0 = no payload stored */
	uint8_t payload[TRACKER_PAYLOAD_MAX];
};

/* Initialize tracker pool (call once at boot). */
void tracker_init(void);

/* Add a new tracked entry with payload for retry.
 * dst_id: who we sent the packet to (upstream).
 * prev_id: who sent it to us (downstream, for backtracking ACK route). Use 0 if N/A.
 * payload/payload_len: the full packet data to resend on timeout. */
int tracker_add(uint16_t dst_id, uint16_t prev_id, uint8_t tracking_id,
		uint8_t packet_type, uint32_t timeout_ms, uint8_t max_retries,
		const void *payload, uint16_t payload_len);

/* Get entry by tracking ID. Returns pointer to entry (without payload), or NULL. */
struct data_tracker *tracker_get_by_tracking_id(uint8_t tracking_id);

/* Get entry by dst_id and packet type. Returns pointer to entry (without payload), or NULL. */
struct data_tracker *tracker_get_by_dst(uint16_t dst_id, uint8_t packet_type);

/* Remove entry by tracking ID. */
void tracker_remove_by_tracking_id(uint8_t tracking_id);

/* Remove entry by dst_id and packet type. */
void tracker_remove_by_dst(uint16_t dst_id, uint8_t packet_type);

/* Remove all entries matching a dst_id. */
void tracker_remove_by_device(uint16_t dst_id);

/* Update payload for an existing tracker entry (by tracking_id). */
int tracker_update_payload(uint8_t tracking_id, const void *payload, uint16_t payload_len);

/* Check all active entries for expiry. For each expired entry, calls
 * the callback with the entry index. If retry is exhausted, the entry
 * is auto-removed before the callback. */
typedef void (*tracker_expired_cb)(int index, struct data_tracker *entry, bool exhausted);
void tracker_tick(tracker_expired_cb cb);

/* Get number of active entries. */
int tracker_active_count(void);

/* Get next tracking ID (1–254, auto-incrementing, circular). */
uint8_t tracker_next_id(void);

/* Default expired callback — retries the packet based on entry->packet_type.
 * Can be passed directly to tracker_tick(). */
void tracker_default_expired_cb(int index, struct data_tracker *entry, bool exhausted);

#endif /* TRACKER_H */
