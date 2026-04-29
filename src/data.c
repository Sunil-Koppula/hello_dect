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

struct data_sender {
	bool     active;
	uint16_t dst_id;
	uint16_t gen_device_id;  /* original origin — preserved across hops */
	uint8_t  priority;
	const uint8_t *buf;
	uint16_t total_size;
	uint8_t  chunk_count;
	uint8_t  last_chunk_size;
	uint8_t  next_chunk;
	uint32_t crc32;
};

static struct data_sender sender;

/* Buffer for staging an inbound transfer's reassembled bytes.
 * Doubles as: (a) the source buffer for ANCHOR forward — must remain valid
 * for the duration of the forward transfer; (b) the CRC verification buffer
 * for any device. Anchors hold this across multiple loop iterations until
 * the forward sender state goes inactive.
 */
static uint8_t reassembly_buf[DATA_MAX_TRANSFER_SIZE];

static uint8_t chunk_size_for(uint8_t chunk_idx)
{
	if (chunk_idx == sender.chunk_count - 1) {
		return sender.last_chunk_size;
	}
	return SEND_DATA_MAX;
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
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_data_init_ack(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_init_ack_t *pkt)
{
	pkt->hdr.packet_type = PACKET_DATA_INIT_ACK;
	pkt->hdr.device_type = PRODUCT_DEVICE_TYPE;
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracking_id;
	pkt->hdr.device_id = dst_id;

	tracker_update_payload(tracking_id, pkt, sizeof(*pkt));
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_data_chunk(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_chunk_t *pkt)
{
	pkt->hdr.packet_type = PACKET_DATA_CHUNK;
	pkt->hdr.device_type = PRODUCT_DEVICE_TYPE;
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracking_id;
	pkt->hdr.device_id = dst_id;

	tracker_update_payload(tracking_id, pkt, sizeof(*pkt));
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_data_chunk_ack(uint16_t dst_id, uint8_t priority, uint8_t tracking_id, data_chunk_ack_t *pkt)
{
	pkt->hdr.packet_type = PACKET_DATA_CHUNK_ACK;
	pkt->hdr.device_type = PRODUCT_DEVICE_TYPE;
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracking_id;
	pkt->hdr.device_id = dst_id;

	tracker_update_payload(tracking_id, pkt, sizeof(*pkt));
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

/* ===== RX handlers ===== */

void handle_data_init(const data_init_t *pkt, uint16_t sender_id, int16_t rssi_2)
{
	if (pkt->hdr.device_id != radio_get_device_id()) {
		return;
	}

	if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY) {
		LOG_WRN("Received DATA_INIT from gateway ID:%d — ignoring", sender_id);
		return;
	}

	LOG_INF("DATA_INIT from %s ID:%d gen %d prio %d size %u chunks %u crc 0x%08x",
		device_type_str(pkt->hdr.device_type), sender_id,
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
		send_data_init_ack(sender_id, pkt->hdr.priority, pkt->hdr.tracking_id, &ack);
		return;
	}

	int idx = find_slot(pkt->gen_device_id, pkt->hdr.priority);
	if (idx < 0) {
		idx = alloc_slot();
		if (idx < 0) {
			LOG_WRN("DATA_INIT rejected: no free slot");
			ack.hdr.status = STATUS_RESOURCE_UNAVAILABLE;
			send_data_init_ack(sender_id, pkt->hdr.priority, pkt->hdr.tracking_id, &ack);
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
	send_data_init_ack(sender_id, pkt->hdr.priority, pkt->hdr.tracking_id, &ack);
}

void handle_data_init_ack(const data_init_ack_t *pkt, uint16_t sender_id, int16_t rssi_2)
{
	if (pkt->hdr.device_id != radio_get_device_id()) {
		return;
	}
	
	LOG_INF("DATA_INIT_ACK from %s ID:%d gen %d prio %d status 0x%02x",
		device_type_str(pkt->hdr.device_type), sender_id,
		pkt->gen_device_id, pkt->hdr.priority, pkt->hdr.status);

	/* Remove Tracker entry. */
	tracker_remove_by_dst(pkt->gen_device_id, PACKET_DATA_INIT);

}

void handle_data_chunk(const data_chunk_t *pkt, uint16_t sender_id, int16_t rssi_2)
{

}

void handle_data_chunk_ack(const data_chunk_ack_t *pkt, uint16_t sender_id, int16_t rssi_2)
{
	
}

/* ===== Module init / tick ===== */

int data_init(void)
{
	for (int i = 0; i < DATA_SLOT_COUNT; i++) {
		slots[i].active = false;
	}
	sender.active = false;

	LOG_INF("Data module: %d slots × %d bytes at PSRAM 0x%06x-0x%06x",
		DATA_SLOT_COUNT, DATA_MAX_TRANSFER_SIZE,
		DATA_PSRAM_BASE, DATA_PSRAM_BASE + DATA_PSRAM_SIZE - 1);

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
