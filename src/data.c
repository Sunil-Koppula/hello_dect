/*
 * Bulk data transfer over the mesh.
 *
 * Receiver side: per-(gen_device_id, priority) reassembly slot in PSRAM.
 * Sender side: single in-flight transfer, sequential chunk-by-chunk with ACK.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include "data.h"
#include "mesh.h"
#include "queue.h"
#include "radio.h"
#include "tracker.h"
#include "timeout.h"
#include "psram.h"
#include "product_info.h"
#include "storage.h"

LOG_MODULE_REGISTER(data, CONFIG_MESH_LOG_LEVEL);

#define DATA_RETRY_TIMEOUT_MS  500
#define DATA_MAX_RETRIES       5

/* ceil(DATA_MAX_TRANSFER_SIZE / SEND_DATA_MAX) — max chunks per transfer. */
#define DATA_MAX_CHUNKS        ((DATA_MAX_TRANSFER_SIZE + SEND_DATA_MAX - 1) / SEND_DATA_MAX)
#define DATA_MASK_BYTES        ((DATA_MAX_CHUNKS + 7) / 8)

/* ===== Receiver state ===== */

struct data_slot {
	bool     active;
	uint16_t gen_device_id;
	uint8_t  priority;
	uint16_t total_size;
	uint8_t  chunk_count;
	uint8_t  last_chunk_size;
	uint32_t crc32;
	uint8_t  received_mask[DATA_MASK_BYTES];
	uint8_t  received_count;
	struct nbtimeout idle_timeout;
};

static struct data_slot slots[DATA_SLOT_COUNT];

static uint32_t slot_psram_addr(int idx)
{
	return DATA_PSRAM_BASE + ((uint32_t)idx * DATA_MAX_TRANSFER_SIZE);
}

static int find_slot(uint16_t gen_device_id, uint8_t priority)
{
	for (int i = 0; i < DATA_SLOT_COUNT; i++) {
		if (slots[i].active &&
		    slots[i].gen_device_id == gen_device_id &&
		    slots[i].priority == priority) {
			return i;
		}
	}
	return -1;
}

static int alloc_slot(void)
{
	for (int i = 0; i < DATA_SLOT_COUNT; i++) {
		if (!slots[i].active) {
			return i;
		}
	}
	return -1;
}

static void slot_free(int idx)
{
	slots[idx].active = false;
	nbtimeout_stop(&slots[idx].idle_timeout);
}

/* ===== Sender state (single in-flight transfer) ===== */

struct data_sender sender;

/* CRC verification staging buffer.
 *
 * On transfer completion we re-read the slot from PSRAM and CRC-check it.
 * Streaming the CRC in small chunks (CRC_VERIFY_STAGE_SIZE bytes at a time)
 * lets us keep the SRAM cost bounded at CRC_VERIFY_STAGE_SIZE rather than
 * the full DATA_MAX_TRANSFER_SIZE. crc32_ieee_update() preserves state
 * across calls, so the per-chunk CRC matches a one-shot crc32_ieee() over
 * the whole buffer.
 *
 * For ANCHOR forward (TODO): the forward path will need to re-read from
 * the same PSRAM slot directly into the outgoing data_chunk_t.data — no
 * full-buffer SRAM staging is required there either.
 */
#define CRC_VERIFY_STAGE_SIZE 256
static uint8_t crc_stage[CRC_VERIFY_STAGE_SIZE];

static uint8_t chunk_size_for(uint8_t chunk_idx)
{
	if (chunk_idx == sender.chunk_count - 1) {
		return sender.last_chunk_size;
	}
	return SEND_DATA_MAX;
}

/* Send sender.next_chunk from PSRAM slot 0, install tracker, advance counter.
 * Returns 0 on success; on failure aborts the transfer (clears sender.active).
 *
 * chunk_pkt is static (~200 B) — only one transfer is in flight at a time, and
 * keeping it off the stack avoids overflowing the main thread when called deep
 * in the RX → handle_data_init_ack → send_next_chunk → tx_large_queue_put chain. */
static data_chunk_t chunk_pkt;

static int send_next_chunk(void)
{
	uint8_t idx = sender.next_chunk;
	uint8_t csz = chunk_size_for(idx);

	memset(&chunk_pkt, 0, sizeof(chunk_pkt));
	chunk_pkt.gen_device_id = sender.gen_device_id;
	chunk_pkt.chunk_index = idx;

	uint32_t addr = slot_psram_addr(0) + (uint32_t)idx * SEND_DATA_MAX;
	int err = psram_read(addr, chunk_pkt.data, csz);
	if (err) {
		LOG_ERR("psram_read @0x%06x failed (%d), aborting transfer", addr, err);
		sender.active = false;
		return err;
	}

	uint8_t tid = tracker_next_id();
	tracker_add(sender.dst_id, radio_get_device_id(), tid, PACKET_DATA_CHUNK,
		    DATA_RETRY_TIMEOUT_MS, DATA_MAX_RETRIES, &chunk_pkt, sizeof(chunk_pkt));
	send_data_chunk(sender.dst_id, sender.priority, tid, &chunk_pkt);
	LOG_INF("Sending DATA_CHUNK to ID:%d (Chunk: %d, Size: %d)", sender.dst_id, idx, csz);
	sender.next_chunk = idx + 1;
	return 0;
}

/* ===== TX builders ===== */

int send_data_init(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_init_t *pkt)
{
	pkt->hdr.packet_type = PACKET_DATA_INIT;
	pkt->hdr.device_type = PRODUCT_DEVICE_TYPE;
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracking_id;
	pkt->hdr.device_id = dst_id;

	tracker_update_payload(tracking_id, pkt, sizeof(*pkt));
	return tx_small_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_data_init_ack(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_init_ack_t *pkt)
{
	pkt->hdr.packet_type = PACKET_DATA_INIT_ACK;
	pkt->hdr.device_type = PRODUCT_DEVICE_TYPE;
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracking_id;
	pkt->hdr.device_id = dst_id;

	return tx_small_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_data_chunk(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_chunk_t *pkt)
{
	pkt->hdr.packet_type = PACKET_DATA_CHUNK;
	pkt->hdr.device_type = PRODUCT_DEVICE_TYPE;
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracking_id;
	pkt->hdr.device_id = dst_id;

	tracker_update_payload(tracking_id, pkt, sizeof(*pkt));
	return tx_large_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_data_chunk_ack(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_chunk_ack_t *pkt)
{
	pkt->hdr.packet_type = PACKET_DATA_CHUNK_ACK;
	pkt->hdr.device_type = PRODUCT_DEVICE_TYPE;
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracking_id;
	pkt->hdr.device_id = dst_id;

	return tx_small_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

/* ===== RX handlers ===== */

void handle_data_init(const data_init_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
	if (pkt->hdr.device_id != radio_get_device_id()) {
		return;
	}

	if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY) {
		LOG_WRN("Received DATA_INIT from gateway ID:%d — ignoring", dst_id);
		return;
	}

	LOG_INF("DATA_INIT from %s ID:%d gen %d prio %d size %u chunks %u crc 0x%08x",
		device_type_str(pkt->hdr.device_type), dst_id,
		pkt->gen_device_id, pkt->hdr.priority,
		pkt->total_size, pkt->chunk_count, pkt->crc32);

	data_init_ack_t ack = {
		.gen_device_id = pkt->gen_device_id,
	};

	if (pkt->total_size == 0 || pkt->total_size > DATA_MAX_TRANSFER_SIZE ||
	    pkt->chunk_count == 0 || pkt->chunk_count > DATA_MAX_CHUNKS ||
	    pkt->last_chunk_size == 0 || pkt->last_chunk_size > SEND_DATA_MAX) {
		LOG_WRN("DATA_INIT rejected: invalid size/chunk params");
		ack.hdr.status = STATUS_INVALID_PARAMETER;
		send_data_init_ack(dst_id, pkt->hdr.priority, pkt->hdr.tracking_id, &ack);
		return;
	}

	int idx = find_slot(pkt->gen_device_id, pkt->hdr.priority);
	if (idx < 0) {
		idx = alloc_slot();
		if (idx < 0) {
			LOG_WRN("DATA_INIT rejected: no free slot");
			ack.hdr.status = STATUS_RESOURCE_UNAVAILABLE;
			send_data_init_ack(dst_id, pkt->hdr.priority, pkt->hdr.tracking_id, &ack);
			return;
		}

		slots[idx].active = true;
		slots[idx].gen_device_id = pkt->gen_device_id;
		slots[idx].priority = pkt->hdr.priority;
		slots[idx].total_size = pkt->total_size;
		slots[idx].chunk_count = pkt->chunk_count;
		slots[idx].last_chunk_size = pkt->last_chunk_size;
		slots[idx].crc32 = pkt->crc32;
		memset(slots[idx].received_mask, 0, sizeof(slots[idx].received_mask));
		slots[idx].received_count = 0;
		nbtimeout_init(&slots[idx].idle_timeout, DATA_SLOT_TIMEOUT_MS, 0);
	}

	nbtimeout_start(&slots[idx].idle_timeout);

	ack.hdr.status = STATUS_SUCCESS;
	send_data_init_ack(dst_id, pkt->hdr.priority, pkt->hdr.tracking_id, &ack);
}

void handle_data_init_ack(const data_init_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
	if (pkt->hdr.device_id != radio_get_device_id()) {
		return;
	}
	
	LOG_INF("DATA_INIT_ACK from %s ID:%d gen %d prio %d status 0x%02x",
		device_type_str(pkt->hdr.device_type), dst_id,
		pkt->gen_device_id, pkt->hdr.priority, pkt->hdr.status);

	// Remove tracker
	tracker_remove_by_dst(dst_id, PACKET_DATA_INIT);
	
	switch (pkt->hdr.device_type) {
		case DEVICE_TYPE_GATEWAY:
			LOG_WRN("Received DATA_INIT_ACK from gateway ID:%d — ignoring", dst_id);
			return;

		case DEVICE_TYPE_ANCHOR:
			LOG_INF("Received DATA_INIT_ACK from %s ID:%d for device %d: status 0x%02x", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id, pkt->hdr.status);
			if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_ANCHOR) {
				// Upstream the Data Chunks if status is success or already exists (in case of retry)
				if (pkt->hdr.status == STATUS_SUCCESS || pkt->hdr.status == STATUS_ALREADY_EXISTS) {
					// get the data from PSRAM to forward
				} else {
					LOG_WRN("DATA_INIT_ACK failed from %s ID:%d for device %d: status 0x%02x", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id, pkt->hdr.status);
				}
			} else if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_SENSOR) {
				// Start sending data chunks if status is success or already exists (in case of retry)
				if (pkt->hdr.status == STATUS_SUCCESS || pkt->hdr.status == STATUS_ALREADY_EXISTS) {
					if (!sender.active || sender.dst_id != dst_id) {
						LOG_WRN("DATA_INIT_ACK from %d but sender inactive or dst mismatch", dst_id);
						return;
					}
					send_next_chunk();
				} else {
					LOG_WRN("DATA_INIT_ACK failed from %s ID:%d for device %d: status 0x%02x", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id, pkt->hdr.status);
				}
			}
			return;
	}

}

void handle_data_chunk(const data_chunk_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
	if (pkt->hdr.device_id != radio_get_device_id()) {
		return;
	}

	if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY) {
		LOG_WRN("Received DATA_CHUNK from gateway ID:%d — ignoring", dst_id);
		return;
	}

	LOG_INF("DATA_CHUNK from %s ID:%d gen %d prio %d chunk %d",
		device_type_str(pkt->hdr.device_type), dst_id,
		pkt->gen_device_id, pkt->hdr.priority, pkt->chunk_index);

	data_chunk_ack_t ack = {
		.gen_device_id = pkt->gen_device_id,
		.chunk_index = pkt->chunk_index,
	};

	int idx = find_slot(pkt->gen_device_id, pkt->hdr.priority);
	if (idx < 0) {
		LOG_WRN("DATA_CHUNK rejected: no slot for gen %d prio %d (DATA_INIT missing or expired)",
			pkt->gen_device_id, pkt->hdr.priority);
		ack.hdr.status = STATUS_NOT_FOUND;
		send_data_chunk_ack(dst_id, pkt->hdr.priority, pkt->hdr.tracking_id, &ack);
		return;
	}

	struct data_slot *s = &slots[idx];

	if (pkt->chunk_index >= s->chunk_count) {
		LOG_WRN("DATA_CHUNK rejected: chunk_index %d out of range (count %d)",
			pkt->chunk_index, s->chunk_count);
		ack.hdr.status = STATUS_INVALID_PARAMETER;
		send_data_chunk_ack(dst_id, pkt->hdr.priority, pkt->hdr.tracking_id, &ack);
		return;
	}

	uint8_t mask_byte = pkt->chunk_index / 8;
	uint8_t mask_bit  = 1u << (pkt->chunk_index % 8);
	bool already = (s->received_mask[mask_byte] & mask_bit) != 0;

	if (!already) {
		uint8_t csz = (pkt->chunk_index == s->chunk_count - 1)
				? s->last_chunk_size : SEND_DATA_MAX;
		uint32_t addr = slot_psram_addr(idx) + (uint32_t)pkt->chunk_index * SEND_DATA_MAX;
		int err = psram_write(addr, pkt->data, csz);
		if (err) {
			LOG_ERR("DATA_CHUNK psram_write @0x%06x failed (%d)", addr, err);
			ack.hdr.status = STATUS_FAILURE;
			send_data_chunk_ack(dst_id, pkt->hdr.priority, pkt->hdr.tracking_id, &ack);
			return;
		}
		s->received_mask[mask_byte] |= mask_bit;
		s->received_count++;
	}

	nbtimeout_start(&s->idle_timeout);
	LOG_INF("Sending DATA_CHUNK_ACK for gen %d prio %d chunk %d to %d (%u/%u)",
		pkt->gen_device_id, pkt->hdr.priority, pkt->chunk_index, dst_id,
		s->received_count, s->chunk_count);
	ack.hdr.status = STATUS_SUCCESS;
	send_data_chunk_ack(dst_id, pkt->hdr.priority, pkt->hdr.tracking_id, &ack);

	if (s->received_count == s->chunk_count) {
		uint32_t base = slot_psram_addr(idx);
		uint32_t crc = 0;
		uint16_t remaining = s->total_size;
		uint16_t off = 0;
		bool first = true;

		while (remaining > 0) {
			uint16_t n = remaining > CRC_VERIFY_STAGE_SIZE
					? CRC_VERIFY_STAGE_SIZE : remaining;
			int err = psram_read(base + off, crc_stage, n);
			if (err) {
				LOG_ERR("Transfer complete but psram_read @0x%06x failed (%d), freeing slot",
					base + off, err);
				slot_free(idx);
				return;
			}
			crc = first ? crc32_ieee(crc_stage, n)
				    : crc32_ieee_update(crc, crc_stage, n);
			first = false;
			off += n;
			remaining -= n;
		}

		if (crc == s->crc32) {
			LOG_INF("Transfer complete: gen %d, %u bytes, CRC OK (0x%08x)",
				s->gen_device_id, s->total_size, crc);
		} else {
			LOG_ERR("Transfer complete: gen %d, %u bytes, CRC FAIL (got 0x%08x exp 0x%08x)",
				s->gen_device_id, s->total_size, crc, s->crc32);
		}
		/* TODO(anchor): if PRODUCT_DEVICE_TYPE == DEVICE_TYPE_ANCHOR, kick off
		 * forward DATA_INIT to the upstream parent here. The forward sender
		 * will pull each chunk straight from this PSRAM slot via psram_read
		 * into the outgoing data_chunk_t.data (no full-buffer SRAM staging).
		 * Defer slot_free() until the forward transfer completes. */
		slot_free(idx);
	}
}

void handle_data_chunk_ack(const data_chunk_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
	if (pkt->hdr.device_id != radio_get_device_id()) {
		return;
	}

	LOG_INF("DATA_CHUNK_ACK from %s ID:%d gen %d chunk %d status 0x%02x",
		device_type_str(pkt->hdr.device_type), dst_id,
		pkt->gen_device_id, pkt->chunk_index, pkt->hdr.status);

	tracker_remove_by_dst(dst_id, PACKET_DATA_CHUNK);

	if (!sender.active || sender.gen_device_id != pkt->gen_device_id) {
		LOG_WRN("DATA_CHUNK_ACK ignored: sender inactive or gen mismatch (got %d, have %d)",
			pkt->gen_device_id, sender.gen_device_id);
		return;
	}

	if (pkt->hdr.status != STATUS_SUCCESS) {
		LOG_ERR("DATA_CHUNK_ACK failed for chunk %d: status 0x%02x, aborting transfer",
			pkt->chunk_index, pkt->hdr.status);
		sender.active = false;
		return;
	}

	if (sender.next_chunk >= sender.chunk_count) {
		LOG_INF("Transfer complete: %u bytes in %u chunks to ID:%d",
			sender.total_size, sender.chunk_count, sender.dst_id);
		sender.active = false;
		return;
	}

	send_next_chunk();
}

/* ===== Module init / tick ===== */

// Helper to build dummy data in PSRAM slot 0 for testing.
// Fills the slot with a counter pattern (byte i = i & 0xFF).
static void build_dummy_data(uint16_t size)
{
	if (sender.active) {
		LOG_WRN("build_dummy_data: sender state is active, cannot build dummy data");
		return;
	}

	if (size == 0 || size > DATA_MAX_TRANSFER_SIZE) {
		LOG_WRN("build_dummy_data: invalid size %u", size);
		return;
	}

	sender.active = true;
	sender.dst_id = PRODUCT_CONNECTED_DEVICE_ID;
	sender.gen_device_id = radio_get_device_id();
	sender.priority = PACKET_PRIORITY_MEDIUM;
	sender.buf = NULL;  // Not used since we're writing directly to PSRAM for testing
	sender.total_size = size;
	sender.chunk_count = (size + SEND_DATA_MAX - 1) / SEND_DATA_MAX;
	sender.last_chunk_size = (size % SEND_DATA_MAX) ? (size % SEND_DATA_MAX) : SEND_DATA_MAX;
	sender.next_chunk = 0;

	uint8_t chunk[256];
	uint32_t base = slot_psram_addr(0);
	uint16_t written = 0;

	while (written < size) {
		uint16_t n = MIN((uint16_t)sizeof(chunk), (uint16_t)(size - written));
		for (uint16_t i = 0; i < n; i++) {
			chunk[i] = (uint8_t)((written + i) & 0xFF);
		}
		if (written == 0) {
			sender.crc32 = crc32_ieee(chunk, n);
		} else {
			sender.crc32 = crc32_ieee_update(sender.crc32, chunk, n);
		}
		int err = psram_write(base + written, chunk, n);
		if (err) {
			LOG_ERR("build_dummy_data: psram_write @0x%06x failed (%d)", base + written, err);
			return;
		}
		written += n;
	}

	LOG_INF("build_dummy_data: wrote %u bytes to PSRAM slot 0 (0x%06x)", size, base);
}

int data_init(void)
{
	for (int i = 0; i < DATA_SLOT_COUNT; i++) {
		slots[i].active = false;
	}
	sender.active = false;

	LOG_INF("Data module: %d slots x %d bytes at PSRAM 0x%06x-0x%06x",
		DATA_SLOT_COUNT, DATA_MAX_TRANSFER_SIZE,
		DATA_PSRAM_BASE, DATA_PSRAM_BASE + DATA_PSRAM_SIZE - 1);

	if (PRODUCT_DEVICE_TYPE == DEVICE_TYPE_SENSOR) {
		LOG_INF("***For testing, building dummy data in PSRAM slot 0 for potential sending...***");
		build_dummy_data(4096);  // For testing: build a 4KB pattern in slot 0 for potential sending.
	}

	return 0;
}

void data_tick(void)
{
	for (int i = 0; i < DATA_SLOT_COUNT; i++) {
		if (slots[i].active && nbtimeout_expired(&slots[i].idle_timeout)) {
			LOG_WRN("Slot %d (gen %d) idle timeout, freeing (%u/%u chunks)",
				i, slots[i].gen_device_id,
				slots[i].received_count, slots[i].chunk_count);
			slot_free(i);
		}
	}
}
