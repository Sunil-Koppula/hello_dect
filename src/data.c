/* Data */

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
#include "main_sub.h"
#include "mesh_layers/mesh_routing.h"

LOG_MODULE_REGISTER(data, CONFIG_DATA_LOG_LEVEL);

#define DATA_MAX_CHUNKS        	((DATA_MAX_TRANSFER_SIZE + SEND_DATA_MAX - 1) / SEND_DATA_MAX)
#define CRC_VERIFY_STAGE_SIZE 	256

static report_chunk_t chunk_pkt;
static uint8_t crc_stage[CRC_VERIFY_STAGE_SIZE];

struct data_sender sender;

/* ===== Receiver state ===== */

struct data_slot {
	bool     active;
	bool	 upstream_ready;
	uint16_t gen_device_id;
	uint8_t data_id;
	uint8_t  priority;
	uint16_t total_size;
	uint8_t  chunk_count;
	uint8_t  last_chunk_size;
	uint32_t crc32;
	bool     received[DATA_MAX_CHUNKS];
	uint8_t  received_count;
	struct nbtimeout idle_timeout;
};

static struct data_slot slots[DATA_SLOT_COUNT];

static uint32_t slot_psram_addr(int idx)
{
	return DATA_PSRAM_BASE + ((uint32_t)idx * DATA_MAX_TRANSFER_SIZE);
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------------**** Data Slots ****------------------------------------------------------------------------------- */

static int find_slot(uint16_t gen_device_id, uint8_t data_id, int *idx_out)
{
	for (int i = 0; i < DATA_SLOT_COUNT; i++) {
		if (slots[i].active && slots[i].gen_device_id == gen_device_id && slots[i].data_id == data_id) {
			if (slots[i].upstream_ready) {
				*idx_out = i;
				return -2; // Special code meaning "slot already active and ready for upstream
			}
			*idx_out = i;
			return i;
		}
	}
	*idx_out = -1;
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

static uint8_t chunk_size_for(uint8_t chunk_idx)
{
	if (chunk_idx == sender.chunk_count - 1) {
		return sender.last_chunk_size;
	}
	return SEND_DATA_MAX;
}

static int send_next_chunk(uint16_t dst_id, uint8_t dst_type)
{
	uint8_t idx = sender.next_chunk;
	uint8_t csz = chunk_size_for(idx);

	memset(&chunk_pkt, 0, sizeof(chunk_pkt));
	chunk_pkt.gen_device_id = sender.gen_device_id;
	chunk_pkt.data_id = sender.data_id;
	chunk_pkt.chunk_index = idx;

	uint32_t addr = slot_psram_addr(0) + (uint32_t)idx * SEND_DATA_MAX;
	int err = psram_read(addr, chunk_pkt.data, csz);
	if (err) {
		LOG_ERR("psram_read @0x%06x failed (%d), aborting transfer", addr, err);
		sender.active = false;
		return err;
	}

	send_report_chunk(&chunk_pkt, dst_id, dst_type, sender.priority);
	sender.next_chunk = idx + 1;
	return 0;
}

static uint8_t validate_slot(const report_init_t *pkt)
{
	int idx;
	int ret = find_slot(pkt->gen_device_id, pkt->data_id, &idx);
	if (ret < 0) {
		if (ret == -2) {
			LOG_WRN("REPORT_INIT rejected: slot already active and ready for upstream for gen %d prio %d", pkt->gen_device_id, pkt->hdr.priority);
			return STATUS_ALREADY_EXISTS;
		}
		idx = alloc_slot();
		if (idx < 0) {
			LOG_WRN("REPORT_INIT rejected: no free slot");
			return STATUS_RESOURCE_UNAVAILABLE;
		}

		slots[idx].active = true;
		slots[idx].upstream_ready = false;
		slots[idx].gen_device_id = pkt->gen_device_id;
		slots[idx].data_id = pkt->data_id;
		slots[idx].priority = pkt->hdr.priority;
		slots[idx].total_size = pkt->total_size;
		slots[idx].chunk_count = pkt->chunk_count;
		slots[idx].last_chunk_size = pkt->last_chunk_size;
		slots[idx].crc32 = pkt->crc32;
		memset(slots[idx].received, 0, sizeof(slots[idx].received));
		slots[idx].received_count = 0;
		nbtimeout_init(&slots[idx].idle_timeout, DATA_SLOT_TIMEOUT_MS, 0);
	}

	nbtimeout_start(&slots[idx].idle_timeout);

	return STATUS_SUCCESS;
}

static uint8_t validate_report_chunk(const report_chunk_t *pkt)
{
	int idx;
	int ret = find_slot(pkt->gen_device_id, pkt->data_id, &idx);
	if (ret < 0) {
		LOG_WRN("REPORT_CHUNK rejected: no active slot for gen %d data_id %d", pkt->gen_device_id, pkt->data_id);
		return STATUS_NOT_FOUND;
	}

	if (pkt->chunk_index >= slots[idx].chunk_count) {
		LOG_WRN("REPORT_CHUNK rejected: invalid chunk index %d (total chunks %d)", pkt->chunk_index, slots[idx].chunk_count);
		return STATUS_INVALID_PARAMETER;
	}

	if (slots[idx].received[pkt->chunk_index]) {
		LOG_WRN("REPORT_CHUNK rejected: chunk index %d already received for gen %d data_id %d", pkt->chunk_index, pkt->gen_device_id, pkt->data_id);
		return STATUS_ALREADY_EXISTS;
	}

	bool already_recieved = slots[idx].received[pkt->chunk_index];

	if(!already_recieved) {
		uint8_t csz = (pkt->chunk_index == slots[idx].chunk_count - 1) ? slots[idx].last_chunk_size : SEND_DATA_MAX;
		uint32_t addr = slot_psram_addr(idx) + (uint32_t)pkt->chunk_index * SEND_DATA_MAX;
		int err = psram_write(addr, pkt->data, csz);
		if (err) {
			LOG_ERR("psram_write @0x%06x failed (%d), aborting transfer", addr, err);
			return STATUS_FAILURE;
		}
		slots[idx].received[pkt->chunk_index] = true;
		slots[idx].received_count++;
	}

	nbtimeout_start(&slots[idx].idle_timeout);

	if (slots[idx].received_count == slots[idx].chunk_count) {
		// All chunks received, verify CRC
		uint32_t base_addr = slot_psram_addr(idx);
		uint32_t crc = 0;
		uint32_t bytes_remaining = slots[idx].total_size;
		uint16_t offset = 0;
		bool first_stage = true;

		while (bytes_remaining > 0) {
			uint16_t n = (bytes_remaining > CRC_VERIFY_STAGE_SIZE) ? CRC_VERIFY_STAGE_SIZE : bytes_remaining;
			int err = psram_read(base_addr + offset, crc_stage, n);
			if (err) {
				LOG_ERR("Transfer complete but psram_read @0x%06x failed (%d), freeing slot", base_addr + offset, err);
				slot_free(idx);
				return STATUS_CRC_FAIL;
			}
			crc = first_stage ? crc32_ieee(crc_stage, n) : crc32_ieee_update(crc, crc_stage, n);
			first_stage = false;
			offset += n;
			bytes_remaining -= n;
		}

		if (crc == slots[idx].crc32) {
			LOG_INF("CRC match for gen %d data_id %d, transfer complete", pkt->gen_device_id, pkt->data_id);
			slots[idx].upstream_ready = true;
			nbtimeout_stop(&slots[idx].idle_timeout);
		} else {
			LOG_ERR("CRC mismatch for gen %d data_id %d: expected 0x%08x computed 0x%08x, freeing slot", pkt->gen_device_id, pkt->data_id, slots[idx].crc32, crc);
			slot_free(idx);
			return STATUS_CRC_FAIL;
		}
	}
	return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------------**** TX Helpers ****------------------------------------------------------------------------------- */

int send_report_init(report_init_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority)
{
	pkt->hdr.packet_type = PACKET_REPORT_INIT;
	pkt->hdr.device_type = get_device_type();
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracker_next_id();
	pkt->hdr.device_id = dst_id;

	// Add tracker entry for retries
	tracker_add(dst_id, get_device_id(), pkt->hdr.tracking_id, PACKET_REPORT_INIT, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, pkt, sizeof(*pkt));

	LOG_INF("----> Sending REPORT_INIT to device %s ID:%d for SENSOR ID:%d", device_type_str(dst_type), dst_id, pkt->gen_device_id);
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_report_init_ack(report_init_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id)
{
	pkt->hdr.packet_type = PACKET_REPORT_INIT_ACK;
	pkt->hdr.device_type = get_device_type();
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracking_id;
	pkt->hdr.device_id = dst_id;

	LOG_INF("----> Sending REPORT_INIT_ACK to device %s ID:%d for SENSOR ID:%d", device_type_str(dst_type), dst_id, pkt->gen_device_id);
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_report_chunk(report_chunk_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority)
{
	pkt->hdr.packet_type = PACKET_REPORT_CHUNK;
	pkt->hdr.device_type = get_device_type();
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracker_next_id();
	pkt->hdr.device_id = dst_id;

	// Add tracker entry for retries
	tracker_add(dst_id, get_device_id(), pkt->hdr.tracking_id, PACKET_REPORT_CHUNK, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, pkt, sizeof(*pkt));

	LOG_INF("----> Sending REPORT_CHUNK to device %s ID:%d for SENSOR ID:%d (Chunk: %d, Size: %d)", device_type_str(dst_type), dst_id, pkt->gen_device_id, pkt->chunk_index, chunk_size_for(pkt->chunk_index));
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_report_chunk_ack(report_chunk_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id)
{
	pkt->hdr.packet_type = PACKET_REPORT_CHUNK_ACK;
	pkt->hdr.device_type = get_device_type();
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracking_id;
	pkt->hdr.device_id = dst_id;

	LOG_INF("----> Sending REPORT_CHUNK_ACK to device %s ID:%d for SENSOR ID:%d (Chunk: %d)", device_type_str(dst_type), dst_id, pkt->gen_device_id, pkt->chunk_index);
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_report_received(report_received_t *pkt, uint16_t dst_id, uint8_t dst_type)
{
	pkt->hdr.packet_type = PACKET_REPORT_RECEIVED;
	pkt->hdr.device_type = get_device_type();
	pkt->hdr.priority = PACKET_PRIORITY_HIGH;
	pkt->hdr.tracking_id = tracker_next_id();
	pkt->hdr.device_id = dst_id;

	// Add tracker entry for retries
	tracker_add(dst_id, get_device_id(), pkt->hdr.tracking_id, PACKET_REPORT_RECEIVED, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, pkt, sizeof(*pkt));

	LOG_INF("----> Sending REPORT_RECEIVED to device %s ID:%d for SENSOR ID:%d data_id %d", device_type_str(dst_type), dst_id, pkt->gen_device_id, pkt->data_id);
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_report_received_ack(report_received_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id)
{
	pkt->hdr.packet_type = PACKET_REPORT_RECEIVED_ACK;
	pkt->hdr.device_type = get_device_type();
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracking_id;
	pkt->hdr.device_id = dst_id;

	LOG_INF("----> Sending REPORT_RECEIVED_ACK to device %s ID:%d for SENSOR ID:%d data_id %d", device_type_str(dst_type), dst_id, pkt->gen_device_id, pkt->data_id);
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------**** Handlers Functions ****--------------------------------------------------------------------------- */

void handle_report_init(const report_init_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
	// Only Process if it's for this device
	if (pkt->hdr.device_id != get_device_id()) {
		return;
	}

	// Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in REPORT_INIT from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

	LOG_INF("Received REPORT_INIT from %s ID:%d for SENSOR ID:%d gen %d prio %d size %u chunks %u crc 0x%08x", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id,
		pkt->gen_device_id, pkt->hdr.priority, pkt->total_size, pkt->chunk_count, pkt->crc32);

	report_init_ack_t ack = {
		.gen_device_id = pkt->gen_device_id,
		.data_id = pkt->data_id,
	};
	
	if (pkt->total_size == 0 || pkt->total_size > DATA_MAX_TRANSFER_SIZE || pkt->chunk_count == 0 || pkt->chunk_count > DATA_MAX_CHUNKS || pkt->last_chunk_size == 0 || pkt->last_chunk_size > SEND_DATA_MAX) {
		LOG_WRN("REPORT_INIT rejected: invalid size/chunk params");
		ack.hdr.status = STATUS_INVALID_PARAMETER;
		send_report_init_ack(&ack, dst_id, pkt->hdr.device_type, pkt->hdr.priority, pkt->hdr.tracking_id);
		return;
	}

	switch (get_device_type()) {
		case DEVICE_TYPE_GATEWAY:
		case DEVICE_TYPE_ANCHOR:
		{
			if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
				// Check there is already an active slot or free slot for this incoming data init, if not reject the packet
				uint8_t status = validate_slot(pkt);
				ack.hdr.status = status;
			} else {
				// Reject data init except from anchor and sensor
				return;
			}
		}
		break;

		case DEVICE_TYPE_SENSOR:
		{
			// Sensor's will not process or transfer any data, so reject any incoming data init 
			return;
		}
		break;

		default:
		{
			// There are only 3 valid device types, reject any data init if this device has invalid type
			return;
		}
		break;

	}
	
	send_report_init_ack(&ack, dst_id, pkt->hdr.device_type, pkt->hdr.priority, pkt->hdr.tracking_id);

	return;
}

void handle_report_init_ack(const report_init_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
	// Only Process if it's for this device
	if (pkt->hdr.device_id != get_device_id()) {
		return;
	}

	// Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in REPORT_INIT_ACK from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }
	
	LOG_INF("Received REPORT_INIT_ACK from %s ID:%d gen %d prio %d status 0x%02x", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id, pkt->hdr.priority, pkt->hdr.status);

	// Remove tracker
	tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

	switch (get_device_type()) {
		case DEVICE_TYPE_GATEWAY:
		{
			// Gateway will never receive report init ack because only anchor and sensor can receive report init ack, so just ignore if received
			return;
		}
		break;

		case DEVICE_TYPE_SENSOR:
		case DEVICE_TYPE_ANCHOR:
		{
			if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
				if (pkt->hdr.status == STATUS_SUCCESS) {
					// Start sending data chunks if status is success
					if (!sender.active || sender.dst_id != dst_id || sender.gen_device_id != pkt->gen_device_id || sender.data_id != pkt->data_id) {
						LOG_WRN("REPORT_INIT_ACK from %d but sender inactive or dst mismatch", dst_id);
						return;
					}
					send_next_chunk(dst_id, pkt->hdr.device_type);
				}
			} else {
				// Reject report init ack except from gateway and anchor
				return;
			}
		}
		break;

		default:
		{
			// There are only 3 valid device types, reject any report init ack if this device has invalid type
			return;
		}
		break;
	}

	return;
}

void handle_report_chunk(const report_chunk_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
	// Only Process if it's for this device
	if (pkt->hdr.device_id != get_device_id()) {
		return;
	}

	// Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in REPORT_CHUNK from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

	LOG_INF("Received REPORT_CHUNK from %s ID:%d gen %d prio %d chunk %d", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id, pkt->hdr.priority, pkt->chunk_index);

	report_chunk_ack_t ack = {
		.gen_device_id = pkt->gen_device_id,
		.data_id = pkt->data_id,
		.chunk_index = pkt->chunk_index,
	};

	switch (get_device_type())
	{
		case DEVICE_TYPE_GATEWAY:
		case DEVICE_TYPE_ANCHOR:
		{
			if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
				// Check the incoming report chunk is valid for an active slot, if not reject the packet
				uint8_t status = validate_report_chunk(pkt);
				ack.hdr.status = status;
			} else {
				// Reject report chunk except from anchor and sensor
				return;
			}
		}
		break;

		case DEVICE_TYPE_SENSOR:
		{
			// Sensor's will not process or transfer any data, so reject any incoming report chunk 
			return;
		}
		
		default:
		{
			// There are only 3 valid device types, reject any report chunk if this device has invalid type
			return;
		}
		break;
	}
	
	send_report_chunk_ack(&ack, dst_id, pkt->hdr.device_type, pkt->hdr.priority, pkt->hdr.tracking_id);

	// Send Data Recieved Packet to Sensor
	int idx;
	int ret = find_slot(pkt->gen_device_id, pkt->data_id, &idx);
	if (get_device_type() == DEVICE_TYPE_GATEWAY && ret == -2) {
		// Implement Report Received Packet.
		LOG_ERR("Need to Send Report Received Packet to Sensor for gen %d data_id %d", pkt->gen_device_id, pkt->data_id);

		{
			// First find route
			uint16_t hop_num = get_hop_num(pkt->gen_device_id, DEVICE_TYPE_SENSOR);
			if (hop_num == 0xFF) {
				LOG_ERR("No route to gen_device_id %d, cannot send REPORT_RECEIVED", pkt->gen_device_id);
				return;
			} else if (hop_num == 0) {
				LOG_INF("Sensor ID:%d is directly connected to this gateway, sending REPORT_RECEIVED without route discovery", pkt->gen_device_id);
				// Build report received packet and send directly without route discovery
				report_received_t recv_pkt = {
					.gen_device_id = pkt->gen_device_id,
					.data_id = pkt->data_id,
					.hdr.status = STATUS_SUCCESS,
				};
				send_report_received(&recv_pkt, pkt->gen_device_id, DEVICE_TYPE_SENSOR);
				return;
			}
			route_discovery_t rd_pkt = {
				.device_id = pkt->gen_device_id,
				.device_type = DEVICE_TYPE_SENSOR,
				.hop_num = hop_num,
				.data_id = pkt->data_id,
				.data_type = DATA_TYPE_REPORT,
			};
			for (int i = 0; i < infra_count; i++) {
				send_route_discovery(&rd_pkt, infra_devices[i].entry.device_id, infra_devices[i].entry.device_type, STATUS_SUCCESS);
			}
		}
	} else if (get_device_type() == DEVICE_TYPE_ANCHOR && ret == -2) {
		infra_entry_t entry;
		int err = storage_infra_get(0, &entry);
		if (err) {
			LOG_ERR("Failed to get infra entry from storage (%d), cannot upstream", err);
			return;
		}
		sender.active = true;
		sender.dst_id = entry.device_id;
		sender.gen_device_id = pkt->gen_device_id;
		sender.data_id = pkt->data_id;
		sender.priority = pkt->hdr.priority;
		sender.total_size = slots[idx].total_size;
		sender.chunk_count = slots[idx].chunk_count;
		sender.last_chunk_size = slots[idx].last_chunk_size;
		sender.crc32 = slots[idx].crc32;
		sender.next_chunk = 0;
		report_init_t init_pkt = {
			.gen_device_id = sender.gen_device_id,
			.data_id = sender.data_id,
			.total_size = sender.total_size,
			.chunk_count = sender.chunk_count,
			.last_chunk_size = sender.last_chunk_size,
			.crc32 = sender.crc32,
		};
		send_report_init(&init_pkt, entry.device_id, entry.device_type, sender.priority);
	}

	return;
}

void handle_report_chunk_ack(const report_chunk_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
	// Only Process if it's for this device
	if (pkt->hdr.device_id != get_device_id()) {
		return;
	}

	// Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in REPORT_CHUNK_ACK from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

	LOG_INF("Received REPORT_CHUNK_ACK from %s ID:%d gen %d chunk %d status 0x%02x", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id, pkt->chunk_index, pkt->hdr.status);

	// Remove tracker
	tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

	// Validate that the ack is for the current active sender transfer, if not ignore the packet
	if (!sender.active || sender.gen_device_id != pkt->gen_device_id || sender.data_id != pkt->data_id || sender.dst_id != dst_id) {
		LOG_WRN("REPORT_CHUNK_ACK ignored: sender inactive or gen/data/dst mismatch (got %d/%d/%d, have %d/%d/%d)", pkt->gen_device_id, pkt->data_id, dst_id, sender.gen_device_id, sender.data_id, sender.dst_id);
		return;
	}

	if (sender.next_chunk >= sender.chunk_count && pkt->hdr.status == STATUS_SUCCESS) {
		LOG_INF("Transfer complete: %u bytes in %u chunks to ID:%d", sender.total_size, sender.chunk_count, sender.dst_id);
		sender.active = false;
		return;
	}

	switch (get_device_type())
	{
		case DEVICE_TYPE_GATEWAY:
		{
			// Gateway will never receive report chunk ack because only anchor and sensor can receive report chunk ack, so just ignore if received
			return;
		}
		break;

		case DEVICE_TYPE_ANCHOR:
		case DEVICE_TYPE_SENSOR:
		{
			if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
				// If status is not found, resend the report init
				if (pkt->hdr.status == STATUS_NOT_FOUND || pkt->hdr.status == STATUS_RESOURCE_UNAVAILABLE || pkt->hdr.status == STATUS_CRC_FAIL) {
					LOG_WRN("Received REPORT_CHUNK_ACK with NOT_FOUND, RESOURCE_UNAVAILABLE, or CRC_FAIL, resending REPORT_INIT");
					report_init_t init_pkt = {
						.gen_device_id = sender.gen_device_id,
						.data_id = sender.data_id,
						.total_size = sender.total_size,
						.chunk_count = sender.chunk_count,
						.last_chunk_size = sender.last_chunk_size,
						.crc32 = sender.crc32,
					};
					send_report_init(&init_pkt, dst_id, pkt->hdr.device_type, sender.priority);
					return;
				} else if (pkt->hdr.status == STATUS_FAILURE || pkt->hdr.status == STATUS_INVALID_PARAMETER) {
					// Rebuild same chunk and resend
					LOG_WRN("Received REPORT_CHUNK_ACK with status 0x%02x, resending chunk %d", pkt->hdr.status, pkt->chunk_index);
					uint8_t idx = pkt->chunk_index;
					uint8_t csz = chunk_size_for(idx);
					memset(&chunk_pkt, 0, sizeof(chunk_pkt));
					chunk_pkt.gen_device_id = sender.gen_device_id;
					chunk_pkt.data_id = sender.data_id;
					chunk_pkt.chunk_index = idx;

					uint32_t addr = slot_psram_addr(0) + (uint32_t)idx * SEND_DATA_MAX;
					int err = psram_read(addr, chunk_pkt.data, csz);
					if (err) {
						LOG_ERR("psram_read @0x%06x failed (%d), aborting transfer", addr, err);
						sender.active = false;
						return;
					}
					send_report_chunk(&chunk_pkt, dst_id, pkt->hdr.device_type, sender.priority);
				} else if (pkt->hdr.status == STATUS_SUCCESS || pkt->hdr.status == STATUS_ALREADY_EXISTS) {
					// Send next chunk if status is success
					send_next_chunk(dst_id, pkt->hdr.device_type);
				}
			} else {
				// Reject report chunk ack except from gateway and anchor
				return;
			}
		}
		break;

		default:
		{
			// There are only 3 valid device types, reject any report chunk ack if this device has invalid type
			return;
		}
		break;
	}

	return;
}

void handle_report_received(const report_received_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
	// Only Process if it's for this device
	if (pkt->hdr.device_id != get_device_id()) {
		return;
	}

	// Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
	if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
		LOG_WRN("Unknown device type 0x%02x in REPORT_RECEIVED from %d, rejecting", pkt->hdr.device_type, dst_id);
		return;
	}

	LOG_INF("Received REPORT_RECEIVED from %s ID:%d gen %d data_id %d", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id, pkt->data_id);

	report_received_ack_t ack = {
		.gen_device_id = pkt->gen_device_id,
		.data_id = pkt->data_id,
	};

	switch (get_device_type()) {
		case DEVICE_TYPE_GATEWAY:
		{
			// Gateway will never receive report received packet because only anchor can receive report received packet, so just ignore if received
			return;
		}
		break;

		case DEVICE_TYPE_ANCHOR:
		{
			if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
				if (pkt->route_info[1].device_id == 0xFFFF) {
					// Check in sensor storage
					for (int i = 0; i < sensor_count; i++) {
						if (sensor_devices[i].entry.device_id == pkt->gen_device_id) {
							report_received_t recv_pkt;
							memcpy(&recv_pkt, pkt, sizeof(report_received_t));
							recv_pkt.route_info[0].device_id = 0xFFFF;
							recv_pkt.route_info[0].hop_num = 0xFF;
							send_report_received(&recv_pkt, pkt->gen_device_id, DEVICE_TYPE_SENSOR);
							ack.hdr.status = STATUS_SUCCESS;
							break;
						}
						if (i == sensor_count - 1) {
							ack.hdr.status = STATUS_NOT_FOUND;
						}
					}
				} else {
					// Forward report received packet to next hop
					report_received_t recv_pkt;
					memcpy(&recv_pkt, pkt, sizeof(report_received_t));
					for (int i = 0; i < MAX_DEPTH - 1; i++) {
						recv_pkt.route_info[i] = recv_pkt.route_info[i + 1];
					}
					recv_pkt.route_info[MAX_DEPTH - 1].device_id = 0xFFFF;
					recv_pkt.route_info[MAX_DEPTH - 1].hop_num = 0xFF;
					ack.hdr.status = STATUS_SUCCESS;
					send_report_received(&recv_pkt, dst_id, DEVICE_TYPE_ANCHOR);
				}
				send_report_received_ack(&ack, dst_id, pkt->hdr.device_type, pkt->hdr.priority, pkt->hdr.tracking_id);
			} else {
				// Reject report received packet except from gateway and anchor
				return;
			}
		}
		break;

		case DEVICE_TYPE_SENSOR:
		{
			if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
				LOG_INF("Gateway Received the Report");
				ack.hdr.status = STATUS_SUCCESS;
				send_report_received_ack(&ack, dst_id, pkt->hdr.device_type, pkt->hdr.priority, pkt->hdr.tracking_id);
				return;
			} else {
				// Reject data received packet except from gateway and anchor
				return;
			}
		}
		break;

		default:
		{
			// There are only 3 valid device types, reject any data received if this device has invalid type
			return;
		}
		break;
	}

	return;
}

void handle_report_received_ack(const report_received_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
	// Only Process if it's for this device
	if (pkt->hdr.device_id != get_device_id()) {
		return;
	}

	// Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
	if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
		LOG_WRN("Unknown device type 0x%02x in REPORT_RECEIVED_ACK from %d, rejecting", pkt->hdr.device_type, dst_id);
		return;
	}

	LOG_INF("Received REPORT_RECEIVED_ACK from %s ID:%d gen %d data_id %d status 0x%02x", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id, pkt->data_id, pkt->hdr.status);

	// Remove tracker
	tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

	switch (get_device_type()) {
		case DEVICE_TYPE_GATEWAY:
		case DEVICE_TYPE_ANCHOR:
		{
			if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
				// No need to process
				return;
			} else {
				// Reject report received ack except from anchor and sensor
				return;
			}
		}
		break;

		case DEVICE_TYPE_SENSOR:
		{
			// Sensor will not send report received ack, so just ignore if received
			return;
		}
		break;

		default:
		{
			// There are only 3 valid device types, reject any data received ack if this device has invalid type
			return;
		}
		break;
	}

	return;
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------**** Module Init / Tick ****--------------------------------------------------------------------------- */

int data_init(void)
{
	for (int i = 0; i < DATA_SLOT_COUNT; i++) {
		slots[i].active = false;
	}
	sender.active = false;

	LOG_INF("Data module Initialized with %d slots (slot size=%d) at PSRAM 0x%06x-0x%06x", DATA_SLOT_COUNT,
		DATA_MAX_TRANSFER_SIZE, DATA_PSRAM_BASE, DATA_PSRAM_BASE + DATA_PSRAM_SIZE - 1);

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