#ifndef TRACKER_H
#define TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include "timeout.h"
#include "product_info.h"

#define TRACKER_MAX_ENTRIES (MAX_SENSORS + MAX_ANCHORS)

/* Tracked device entry. */
struct data_tracker {
	uint16_t device_id;
	uint8_t tracking_id;
	struct nbtimeout timeout;
	bool active;
};

/* Initialize tracker pool (call once at boot). */
void tracker_init(void);

/* Add a new tracked entry. Returns index on success, -1 if pool full. */
int tracker_add(uint16_t device_id, uint8_t tracking_id,
		uint32_t timeout_ms, uint8_t max_retries);

/* Find entry by device ID. Returns index, or -1 if not found. */
int tracker_find_by_device(uint16_t device_id);

/* Find entry by tracking ID. Returns index, or -1 if not found. */
int tracker_find_by_tracking_id(uint8_t tracking_id);

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

#endif /* TRACKER_H */
