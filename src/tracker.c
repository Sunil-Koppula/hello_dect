/*
 * Device data tracker for DECT NR+ mesh network.
 *
 * Index (metadata + timeout) in internal RAM for fast tick/find.
 * Payload data stored in PSRAM with CRC16 for integrity checking.
 *
 * PSRAM slot layout per entry:
 *   [payload (up to 210 bytes)] [CRC16 (2 bytes)]
 *   Slot size = TRACKER_PAYLOAD_MAX + 2 = 212 bytes
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include "tracker.h"
#include "queue.h"
#include "protocol.h"
#include "mesh.h"
#include "psram.h"

LOG_MODULE_REGISTER(tracker, CONFIG_TRACKER_LOG_LEVEL);

/* Each PSRAM slot: payload + 2-byte CRC16. */
#define TRACKER_SLOT_SIZE (TRACKER_PAYLOAD_MAX + sizeof(uint16_t))

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

/* PSRAM address for a given slot. */
static inline uint32_t psram_slot_addr(int idx)
{
	return TRACKER_PSRAM_BASE + (idx * TRACKER_SLOT_SIZE);
}

/* Temp buffer for PSRAM read (payload + CRC). */
static uint8_t psram_buf[TRACKER_SLOT_SIZE];

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

/* Write payload + CRC16 to PSRAM. */
static int psram_payload_write(int idx, const void *payload, uint16_t len)
{
	if (!is_psram_initialized()) {
		return -ENODEV;
	}

	uint16_t crc = crc16_ccitt(0xFFFF, payload, len);

	/* Write payload. */
	int err = psram_write(psram_slot_addr(idx), payload, len);
	if (err) {
		return err;
	}

	/* Write CRC16 right after payload. */
	return psram_write(psram_slot_addr(idx) + len, &crc, sizeof(crc));
}

/* Read payload from PSRAM and verify CRC16.
 * Returns 0 on success, -EILSEQ on CRC mismatch, or negative errno. */
static int psram_payload_read(int idx, uint8_t *out, uint16_t len)
{
	if (!is_psram_initialized()) {
		return -ENODEV;
	}

	/* Read payload + CRC in one go. */
	uint16_t read_len = len + sizeof(uint16_t);

	int err = psram_read(psram_slot_addr(idx), psram_buf, read_len);
	if (err) {
		return err;
	}

	/* Verify CRC16. */
	uint16_t stored_crc;
	memcpy(&stored_crc, &psram_buf[len], sizeof(stored_crc));

	uint16_t computed_crc = crc16_ccitt(0xFFFF, psram_buf, len);

	if (stored_crc != computed_crc) {
		LOG_ERR("PSRAM CRC mismatch idx %d: stored=0x%04x computed=0x%04x",
			idx, stored_crc, computed_crc);
		return -EILSEQ;
	}

	memcpy(out, psram_buf, len);
	return 0;
}

void tracker_init(void)
{
	for (int i = 0; i < TRACKER_MAX_ENTRIES; i++) {
		index_pool[i].active = false;
	}

	next_tracking_id = 1;

	LOG_INF("Tracker init: %d entries, PSRAM payload 0x%06x–0x%06x (%d bytes, slot=%d)",
		TRACKER_MAX_ENTRIES, TRACKER_PSRAM_BASE,
		TRACKER_PSRAM_BASE + TRACKER_PSRAM_SIZE - 1, TRACKER_PSRAM_SIZE,
		TRACKER_SLOT_SIZE);
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
				psram_payload_write(i, payload, payload_len);
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
			return psram_payload_write(i, payload, payload_len);
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

			/* Load payload from PSRAM with CRC verification. */
			if (entry->payload_len > 0) {
				int err = psram_payload_read(i, temp_entry.payload, entry->payload_len);
				if (err) {
					LOG_ERR("Payload read failed for idx %d (err %d), skipping retry",
						i, err);
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
