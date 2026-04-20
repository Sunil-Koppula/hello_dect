/*
 * Device data tracker for DECT NR+ mesh network.
 *
 * Manages a pool of active device+timeout pairs for tracking
 * outstanding operations (data sends, pair handshakes, etc.).
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "tracker.h"
#include "queue.h"
#include "protocol.h"
#include "mesh.h"

LOG_MODULE_REGISTER(tracker, CONFIG_TRACKER_LOG_LEVEL);

static struct data_tracker pool[TRACKER_MAX_ENTRIES];
static uint8_t next_tracking_id = 1;

void tracker_init(void)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		pool[i].active = false;
	}

	next_tracking_id = 1;
}

int tracker_add(uint16_t device_id, uint16_t src_id, uint8_t tracking_id,
		uint8_t packet_type, uint32_t timeout_ms, uint8_t max_retries,
		const void *payload, uint16_t payload_len)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (!pool[i].active) {
			pool[i].device_id = device_id;
			pool[i].src_id = src_id;
			pool[i].tracking_id = tracking_id;
			pool[i].packet_type = packet_type;
			pool[i].active = true;
			pool[i].payload_len = 0;

			if (payload && payload_len > 0) {
				if (payload_len > TRACKER_PAYLOAD_MAX) {
					payload_len = TRACKER_PAYLOAD_MAX;
				}
				memcpy(pool[i].payload, payload, payload_len);
				pool[i].payload_len = payload_len;
			}

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

int tracker_find_by_tracking_id(uint8_t tracking_id)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (pool[i].active && pool[i].tracking_id == tracking_id) {
			return i;
		}
	}

	return -1;
}

int tracker_find(uint16_t device_id, uint8_t packet_type)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (pool[i].active &&
		    pool[i].device_id == device_id &&
		    pool[i].packet_type == packet_type) {
			return i;
		}
	}

	return -1;
}

uint16_t tracker_get_src_id(uint16_t device_id, uint8_t packet_type)
{
	int idx = tracker_find(device_id, packet_type);

	if (idx >= 0) {
		return pool[idx].src_id;
	}

	return 0;
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
			entry->tracking_id, entry->packet_type, entry->device_id);
		return;
	}

	LOG_INF("Tracker 0x%02x (pkt: 0x%02x) for device %d retry %d/%d",
		entry->tracking_id, entry->packet_type, entry->device_id,
		nbtimeout_retries(&entry->timeout),
		nbtimeout_max_retries(&entry->timeout));

	if (entry->payload_len > 0) {
		/* Resend stored payload directly. */
		tx_queue_put(entry->payload, entry->payload_len, QUEUE_PRIO_HIGH);
	} else {
		/* No payload stored — rebuild from tracker fields. */
		LOG_WRN("No payload for retry, rebuilding packet for type 0x%02x", entry->packet_type);
		switch (entry->packet_type) {
		case PACKET_PAIR_REQUEST:
			send_pair_request(0, entry->tracking_id);
			break;
		case PACKET_PAIR_CONFIRM:
			send_pair_confirm(0, entry->device_id, entry->tracking_id, STATUS_SUCCESS);
			break;
		default:
			LOG_WRN("No payload for packet type 0x%02x retry, cannot resend",
				entry->packet_type);
			break;
		}
	}
}
