/*
 * Fragmentation and reassembly for DECT NR+ mesh network.
 *
 * TX: splits large data into fragments, each queued with a 3-byte header.
 * RX: collects fragments by frag_id + sender_id, returns complete data.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "fragment.h"
#include "queue.h"

LOG_MODULE_REGISTER(fragment, CONFIG_FRAGMENT_LOG_LEVEL);

static uint8_t next_frag_id;
static struct frag_reassembly reassembly[FRAG_REASSEMBLY_SLOTS];

void fragment_init(void)
{
	next_frag_id = 0;

	for (int i = 0; i < FRAG_REASSEMBLY_SLOTS; i++) {
		reassembly[i].active = false;
	}
}

uint8_t fragment_next_id(void)
{
	return next_frag_id++;
}

int fragment_send(const void *data, size_t data_len, uint8_t subslots)
{
	if (subslots == 0 || subslots > FRAG_MAX_SUBSLOTS) {
		subslots = FRAG_DEFAULT_SUBSLOTS;
	}

	size_t max_payload = (subslots * FRAG_BYTES_PER_SUBSLOT) - FRAG_HEADER_SIZE;
	uint8_t frag_total = (data_len + max_payload - 1) / max_payload;

	if (frag_total == 0) {
		frag_total = 1;
	}

	if (frag_total > 255) {
		LOG_ERR("Data too large to fragment (%d bytes, max %d)",
			data_len, max_payload * 255);
		return -ENOMEM;
	}

	uint8_t frag_id = fragment_next_id();
	const uint8_t *src = (const uint8_t *)data;
	size_t remaining = data_len;

	LOG_DBG("Fragmenting %d bytes into %d fragments (id: %d, payload: %d)",
		data_len, frag_total, frag_id, max_payload);

	for (uint8_t i = 0; i < frag_total; i++) {
		size_t chunk = (remaining > max_payload) ? max_payload : remaining;
		uint8_t buf[FRAG_HEADER_SIZE + max_payload];

		/* Prepend fragment header. */
		buf[0] = frag_id;
		buf[1] = i;
		buf[2] = frag_total;
		memcpy(&buf[FRAG_HEADER_SIZE], src, chunk);

		int err = tx_queue_put(buf, FRAG_HEADER_SIZE + chunk, QUEUE_PRIO_LOW);

		if (err) {
			LOG_ERR("Failed to queue fragment %d/%d", i + 1, frag_total);
			return err;
		}

		src += chunk;
		remaining -= chunk;
	}

	return 0;
}

const uint8_t *fragment_receive(const uint8_t *data, size_t data_len,
				uint16_t sender_id, size_t *out_len)
{
	if (data_len < FRAG_HEADER_SIZE) {
		return NULL;
	}

	const struct frag_header *hdr = (const struct frag_header *)data;
	const uint8_t *payload = data + FRAG_HEADER_SIZE;
	size_t payload_len = data_len - FRAG_HEADER_SIZE;

	/* Single fragment — no reassembly needed. */
	if (hdr->frag_total == 1) {
		*out_len = payload_len;
		return payload;
	}

	if (hdr->frag_index >= hdr->frag_total) {
		LOG_WRN("Invalid fragment index %d/%d", hdr->frag_index, hdr->frag_total);
		return NULL;
	}

	/* Find existing reassembly slot. */
	struct frag_reassembly *slot = NULL;

	for (int i = 0; i < FRAG_REASSEMBLY_SLOTS; i++) {
		if (reassembly[i].active &&
		    reassembly[i].frag_id == hdr->frag_id &&
		    reassembly[i].sender_id == sender_id) {
			slot = &reassembly[i];
			break;
		}
	}

	/* Allocate new slot if not found. */
	if (!slot) {
		for (int i = 0; i < FRAG_REASSEMBLY_SLOTS; i++) {
			if (!reassembly[i].active) {
				slot = &reassembly[i];
				slot->active = true;
				slot->frag_id = hdr->frag_id;
				slot->sender_id = sender_id;
				slot->frag_total = hdr->frag_total;
				slot->received_mask = 0;
				slot->total_len = 0;
				break;
			}
		}
	}

	if (!slot) {
		LOG_WRN("No reassembly slots available");
		return NULL;
	}

	/* Calculate max payload per fragment from total. */
	size_t max_payload = (FRAG_DEFAULT_SUBSLOTS * FRAG_BYTES_PER_SUBSLOT) - FRAG_HEADER_SIZE;

	/* Store fragment at correct offset. */
	size_t offset = hdr->frag_index * max_payload;

	if (offset + payload_len > sizeof(slot->buf)) {
		LOG_WRN("Fragment would overflow reassembly buffer");
		slot->active = false;
		return NULL;
	}

	memcpy(&slot->buf[offset], payload, payload_len);
	slot->received_mask |= (1 << hdr->frag_index);

	/* Track total length (last fragment determines actual total). */
	if (offset + payload_len > slot->total_len) {
		slot->total_len = offset + payload_len;
	}

	LOG_DBG("Fragment %d/%d (id:%d) from %d, mask: 0x%02x",
		hdr->frag_index + 1, hdr->frag_total, hdr->frag_id,
		sender_id, slot->received_mask);

	/* Check if all fragments received. */
	uint8_t expected_mask = (1 << hdr->frag_total) - 1;

	if (slot->received_mask == expected_mask) {
		*out_len = slot->total_len;
		slot->active = false;
		return slot->buf;
	}

	return NULL;
}
