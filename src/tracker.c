/*
 * Device data tracker for DECT NR+ mesh network.
 *
 * Index (metadata + timeout) in internal RAM for fast tick/find.
 * Payload data stored in PSRAM to save RAM.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "tracker.h"
#include "queue.h"
#include "protocol.h"
#include "mesh.h"
#include "psram.h"

LOG_MODULE_REGISTER(tracker, CONFIG_TRACKER_LOG_LEVEL);

/* RAM index — everything except payload. */
struct tracker_index {
	uint16_t dst_id;
	uint16_t prev_id;
	uint8_t tracking_id;
	uint8_t packet_type;
	struct nbtimeout timeout;
	bool active;
	uint16_t payload_len;
};

static struct tracker_index index_pool[TRACKER_MAX_ENTRIES];
static uint8_t next_tracking_id = 1;

/* PSRAM layout: one payload buffer per slot. */
static inline uint32_t psram_payload_addr(int idx)
{
	return TRACKER_PSRAM_BASE + (idx * TRACKER_PAYLOAD_MAX);
}

/* Temp buffer for reading payload from PSRAM. */
static uint8_t payload_buf[TRACKER_PAYLOAD_MAX];

/* Fill a data_tracker from the RAM index (no payload). */
static struct data_tracker temp_entry;

static struct data_tracker *fill_from_index(int idx)
{
	temp_entry.dst_id = index_pool[idx].dst_id;
	temp_entry.prev_id = index_pool[idx].prev_id;
	temp_entry.tracking_id = index_pool[idx].tracking_id;
	temp_entry.packet_type = index_pool[idx].packet_type;
	temp_entry.timeout = index_pool[idx].timeout;
	temp_entry.active = index_pool[idx].active;
	temp_entry.payload_len = index_pool[idx].payload_len;
	return &temp_entry;
}

void tracker_init(void)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		index_pool[i].active = false;
	}

	next_tracking_id = 1;

	LOG_INF("Tracker init: %d entries, payload in PSRAM 0x%06x–0x%06x (%d bytes)",
		TRACKER_MAX_ENTRIES, TRACKER_PSRAM_BASE,
		TRACKER_PSRAM_BASE + TRACKER_PSRAM_SIZE - 1, TRACKER_PSRAM_SIZE);
}

int tracker_add(uint16_t dst_id, uint16_t prev_id, uint8_t tracking_id,
		uint8_t packet_type, uint32_t timeout_ms, uint8_t max_retries,
		const void *payload, uint16_t payload_len)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (!index_pool[i].active) {
			index_pool[i].dst_id = dst_id;
			index_pool[i].prev_id = prev_id;
			index_pool[i].tracking_id = tracking_id;
			index_pool[i].packet_type = packet_type;
			index_pool[i].active = true;
			index_pool[i].payload_len = 0;
			nbtimeout_init(&index_pool[i].timeout, timeout_ms, max_retries);
			nbtimeout_start(&index_pool[i].timeout);

			if (payload && payload_len > 0) {
				if (payload_len > TRACKER_PAYLOAD_MAX) {
					payload_len = TRACKER_PAYLOAD_MAX;
				}
				index_pool[i].payload_len = payload_len;

				if (is_psram_initialized()) {
					psram_write(psram_payload_addr(i), payload, payload_len);
				}
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
		if (index_pool[i].active && index_pool[i].tracking_id == tracking_id) {
			return fill_from_index(i);
		}
	}

	return NULL;
}

struct data_tracker *tracker_get_by_dst(uint16_t dst_id, uint8_t packet_type)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (index_pool[i].active &&
		    index_pool[i].dst_id == dst_id &&
		    index_pool[i].packet_type == packet_type) {
			return fill_from_index(i);
		}
	}

	return NULL;
}

void tracker_remove_by_tracking_id(uint8_t tracking_id)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (index_pool[i].active && index_pool[i].tracking_id == tracking_id) {
			index_pool[i].active = false;
			nbtimeout_stop(&index_pool[i].timeout);
			return;
		}
	}
}

void tracker_remove_by_dst(uint16_t dst_id, uint8_t packet_type)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (index_pool[i].active &&
		    index_pool[i].dst_id == dst_id &&
		    index_pool[i].packet_type == packet_type) {
			index_pool[i].active = false;
			nbtimeout_stop(&index_pool[i].timeout);
			return;
		}
	}
}

void tracker_remove_by_device(uint16_t dst_id)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (index_pool[i].active && index_pool[i].dst_id == dst_id) {
			index_pool[i].active = false;
			nbtimeout_stop(&index_pool[i].timeout);
		}
	}
}

int tracker_update_payload(uint8_t tracking_id, const void *payload, uint16_t payload_len)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (index_pool[i].active && index_pool[i].tracking_id == tracking_id) {
			if (payload_len > TRACKER_PAYLOAD_MAX) {
				payload_len = TRACKER_PAYLOAD_MAX;
			}
			index_pool[i].payload_len = payload_len;

			if (is_psram_initialized()) {
				return psram_write(psram_payload_addr(i), payload, payload_len);
			}

			return 0;
		}
	}

	LOG_WRN("Tracker 0x%02x not found for payload update", tracking_id);
	return -ENOENT;
}

void tracker_tick(tracker_expired_cb cb)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (!index_pool[i].active) {
			continue;
		}

		if (!nbtimeout_expired(&index_pool[i].timeout)) {
			continue;
		}

		bool exhausted = nbtimeout_retry(&index_pool[i].timeout);

		if (exhausted) {
			index_pool[i].active = false;
		}

		if (cb) {
			struct data_tracker *entry = fill_from_index(i);

			/* Load payload from PSRAM if available. */
			if (entry->payload_len > 0 && is_psram_initialized()) {
				if (psram_read(psram_payload_addr(i), payload_buf, entry->payload_len) == 0) {
					memcpy(temp_entry.payload, payload_buf, entry->payload_len);
				} else {
					LOG_ERR("Failed to read payload from PSRAM for idx %d", i);
					temp_entry.payload_len = 0;
				}
			}

			cb(i, entry, exhausted);
		}
	}
}

int tracker_active_count(void)
{
	int count = 0;

	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		if (index_pool[i].active) {
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
