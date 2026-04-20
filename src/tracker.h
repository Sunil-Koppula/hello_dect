#ifndef TRACKER_H
#define TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include "timeout.h"
#include "product_info.h"
#include "queue.h"

#define TRACKER_MAX_ENTRIES (MAX_SENSORS + MAX_ANCHORS)
#define TRACKER_PAYLOAD_MAX QUEUE_DATA_MAX

/* Tracked device entry. */
struct data_tracker {
	uint16_t device_id;     /* who we sent the packet TO (upstream target) */
	uint16_t src_id;        /* who sent the packet TO US (downstream source, for backtracking) */
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
 * device_id: who we sent the packet to (upstream).
 * src_id: who sent it to us (downstream, for backtracking ACK route). Use 0 if N/A.
 * payload/payload_len: the full packet data to resend on timeout. */
int tracker_add(uint16_t device_id, uint16_t src_id, uint8_t tracking_id,
		uint8_t packet_type, uint32_t timeout_ms, uint8_t max_retries,
		const void *payload, uint16_t payload_len);

/* Find entry by device ID. Returns index, or -1 if not found. */
int tracker_find_by_device(uint16_t device_id);

/* Find entry by tracking ID. Returns index, or -1 if not found. */
int tracker_find_by_tracking_id(uint8_t tracking_id);

/* Find entry by device ID and packet type. Returns index, or -1 if not found. */
int tracker_find(uint16_t device_id, uint8_t packet_type);

/* Get src_id (downstream source) by device_id and packet_type. Returns 0 if not found. */
uint16_t tracker_get_src_id(uint16_t device_id, uint8_t packet_type);

/* Get entry by index. Returns NULL if index invalid or inactive. */
struct data_tracker *tracker_get(int index);

/* Remove (deactivate) entry by index. */
void tracker_remove(int index);

/* Remove all entries matching a device ID. */
void tracker_remove_by_device(uint16_t device_id);

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
