/*
 * Device data tracker for DECT NR+ mesh network.
 *
 * Manages a pool of active device+timeout pairs for tracking
 * outstanding operations (data sends, pair handshakes, etc.).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "tracker.h"

LOG_MODULE_REGISTER(tracker, CONFIG_MAIN_LOG_LEVEL);

static struct data_tracker pool[TRACKER_MAX_ENTRIES];

void tracker_init(void)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		pool[i].active = false;
	}
}

int tracker_add(uint16_t device_id, uint16_t tracking_id,
		uint32_t timeout_ms, uint8_t max_retries)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (!pool[i].active) {
			pool[i].device_id = device_id;
			pool[i].tracking_id = tracking_id;
			pool[i].active = true;
			nbtimeout_init(&pool[i].timeout, timeout_ms, max_retries);
			nbtimeout_start(&pool[i].timeout);
			return i;
		}
	}

	LOG_WRN("Tracker pool full");
	return -1;
}

int tracker_find_by_device(uint16_t device_id)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (pool[i].active && pool[i].device_id == device_id) {
			return i;
		}
	}

	return -1;
}

int tracker_find_by_tracking_id(uint16_t tracking_id)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (pool[i].active && pool[i].tracking_id == tracking_id) {
			return i;
		}
	}

	return -1;
}

struct data_tracker *tracker_get(int index)
{
	if (index < 0 || index >= TRACKER_MAX_ENTRIES) {
		return NULL;
	}

	if (!pool[index].active) {
		return NULL;
	}

	return &pool[index];
}

void tracker_remove(int index)
{
	if (index >= 0 && index < TRACKER_MAX_ENTRIES) {
		pool[index].active = false;
		nbtimeout_stop(&pool[index].timeout);
	}
}

void tracker_remove_by_device(uint16_t device_id)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (pool[i].active && pool[i].device_id == device_id) {
			pool[i].active = false;
			nbtimeout_stop(&pool[i].timeout);
		}
	}
}

void tracker_tick(tracker_expired_cb cb)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (!pool[i].active) {
			continue;
		}

		if (!nbtimeout_expired(&pool[i].timeout)) {
			continue;
		}

		bool exhausted = nbtimeout_retry(&pool[i].timeout);

		if (exhausted) {
			pool[i].active = false;
		}

		if (cb) {
			cb(i, &pool[i], exhausted);
		}
	}
}

int tracker_active_count(void)
{
	int count = 0;

	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (pool[i].active) {
			count++;
		}
	}

	return count;
}
