/*
 * Device data tracker for DECT NR+ mesh network.
 *
 * All entries stored in internal RAM.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "tracker.h"
#include "queue.h"
#include "protocol.h"
#include "mesh.h"

LOG_MODULE_REGISTER(tracker, CONFIG_TRACKER_LOG_LEVEL);

static struct data_tracker entry_pool[TRACKER_MAX_ENTRIES];
static uint8_t next_tracking_id = 1;

void tracker_init(void)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		entry_pool[i].active = false;
	}

	next_tracking_id = 1;
}

int tracker_add(uint16_t dst_id, uint16_t prev_id, uint8_t tracking_id,
		uint8_t packet_type, uint32_t timeout_ms, uint8_t max_retries,
		const void *payload, uint16_t payload_len)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (!entry_pool[i].active) {
			entry_pool[i].dst_id = dst_id;
			entry_pool[i].prev_id = prev_id;
			entry_pool[i].tracking_id = tracking_id;
			entry_pool[i].packet_type = packet_type;
			entry_pool[i].active = true;
			entry_pool[i].payload_len = 0;
			nbtimeout_init(&entry_pool[i].timeout, timeout_ms, max_retries);
			nbtimeout_start(&entry_pool[i].timeout);

			if (payload && payload_len > 0) {
				if (payload_len > TRACKER_PAYLOAD_MAX) {
					payload_len = TRACKER_PAYLOAD_MAX;
				}
				memcpy(entry_pool[i].payload, payload, payload_len);
				entry_pool[i].payload_len = payload_len;
			}

			return i;
		}
	}

	LOG_WRN("Tracker pool full");
	return -1;
}

struct data_tracker *tracker_get_by_tracking_id(uint8_t tracking_id)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (entry_pool[i].active && entry_pool[i].tracking_id == tracking_id) {
			return &entry_pool[i];
		}
	}

	return NULL;
}

struct data_tracker *tracker_get_by_dst(uint16_t dst_id, uint8_t packet_type)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (entry_pool[i].active &&
		    entry_pool[i].dst_id == dst_id &&
		    entry_pool[i].packet_type == packet_type) {
			return &entry_pool[i];
		}
	}

	return NULL;
}

void tracker_remove_by_tracking_id(uint8_t tracking_id)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (entry_pool[i].active && entry_pool[i].tracking_id == tracking_id) {
			entry_pool[i].active = false;
			nbtimeout_stop(&entry_pool[i].timeout);
			return;
		}
	}
}

void tracker_remove_by_dst(uint16_t dst_id, uint8_t packet_type)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (entry_pool[i].active &&
		    entry_pool[i].dst_id == dst_id &&
		    entry_pool[i].packet_type == packet_type) {
			entry_pool[i].active = false;
			nbtimeout_stop(&entry_pool[i].timeout);
			return;
		}
	}
}

void tracker_remove_by_device(uint16_t dst_id)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (entry_pool[i].active && entry_pool[i].dst_id == dst_id) {
			entry_pool[i].active = false;
			nbtimeout_stop(&entry_pool[i].timeout);
		}
	}
}

int tracker_update_payload(uint8_t tracking_id, const void *payload, uint16_t payload_len)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (entry_pool[i].active && entry_pool[i].tracking_id == tracking_id) {
			if (payload_len > TRACKER_PAYLOAD_MAX) {
				payload_len = TRACKER_PAYLOAD_MAX;
			}
			memcpy(entry_pool[i].payload, payload, payload_len);
			entry_pool[i].payload_len = payload_len;
			return 0;
		}
	}

	LOG_WRN("Tracker 0x%02x not found for payload update", tracking_id);
	return -ENOENT;
}

void tracker_tick(tracker_expired_cb cb)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (!entry_pool[i].active) {
			continue;
		}

		if (!nbtimeout_expired(&entry_pool[i].timeout)) {
			continue;
		}

		bool exhausted = nbtimeout_retry(&entry_pool[i].timeout);

		if (exhausted) {
			entry_pool[i].active = false;
		}

		if (cb) {
			cb(i, &entry_pool[i], exhausted);
		}
	}
}

int tracker_active_count(void)
{
	int count = 0;

	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (entry_pool[i].active) {
			count++;
		}
	}

	return count;
}

uint8_t tracker_next_id(void)
{
	uint8_t id = next_tracking_id;

	next_tracking_id++;
	if (next_tracking_id >= 255) {
		next_tracking_id = 1;
	}

	return id;
}

void tracker_default_expired_cb(int index, struct data_tracker *entry, bool exhausted)
{
	if (exhausted) {
		LOG_WRN("Tracker 0x%02x (pkt: 0x%02x) for device %d exhausted",
			entry->tracking_id, entry->packet_type, entry->dst_id);
		return;
	}

	LOG_INF("Tracker 0x%02x (pkt: 0x%02x) for device %d retry %d/%d",
		entry->tracking_id, entry->packet_type, entry->dst_id,
		nbtimeout_retries(&entry->timeout),
		nbtimeout_max_retries(&entry->timeout));

	if (entry->payload_len > 0) {
		tx_queue_put(entry->payload, entry->payload_len, entry->payload[2]);
	} else {
		LOG_WRN("No payload for packet type 0x%02x retry, cannot resend",
			entry->packet_type);
	}
}
