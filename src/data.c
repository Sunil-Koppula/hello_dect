/* Data */

#include <string.h>
#include <zephyr/kernel.h>
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
#include "slm_at_main.h"
#include "slm_at_uart.h"
#include "mesh_layers/mesh_routing.h"
#include "log_color.h"

LOG_MODULE_REGISTER(data, CONFIG_DATA_LOG_LEVEL);

static report_chunk_t chunk_pkt;
static uint8_t crc_stage[CRC_VERIFY_STAGE_SIZE];

struct report_sender report_sender[MAX_ANCHORS];
struct report_slot report_slot[DATA_SLOT_COUNT];

static uint16_t high_prio_report_count = 0;
static uint16_t med_prio_report_count = 0;
static uint16_t low_prio_report_count = 0;

static uint32_t slot_psram_addr(int idx)
{
	return DATA_PSRAM_BASE + ((uint32_t)idx * MAX_REPORT_SIZE);
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------**** Report Sender ****----------------------------------------------------------------------------- */
static int find_sender_slot(uint16_t dst_id, uint8_t data_id)
{
	for (int i = 0; i < MAX_ANCHORS; i++) {
		if (report_sender[i].active && report_sender[i].gen_device_id == dst_id && report_sender[i].data_id == data_id) {
			return i;
		}
	}
	return -1;
}

static void free_sender_slot(int idx)
{
	report_sender[idx].active = false;
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------**** Report Slots ****------------------------------------------------------------------------------ */

static int find_slot(uint16_t gen_device_id, uint8_t data_id)
{
	for (int i = 0; i < DATA_SLOT_COUNT; i++) {
		if (report_slot[i].active && report_slot[i].gen_device_id == gen_device_id && report_slot[i].data_id == data_id) {
			return i;
		}
	}
	return -1;
}

static int alloc_slot(void)
{
	for (int i = 0; i < DATA_SLOT_COUNT; i++) {
		if (!report_slot[i].active) {
			report_slot[i].is_sent = false;
			report_slot[i].upstream_ready = false;
			report_slot[i].is_transfered = false;
			return i;
		}
	}
	return -1;
}

static void update_report_count(uint8_t priority, bool is_increment)
{
	switch (priority) {
		case PACKET_PRIORITY_HIGH:
		{
			if (high_prio_report_count < UINT16_MAX && is_increment) {
				high_prio_report_count++;
			} else if (!is_increment && high_prio_report_count > 0) {
				high_prio_report_count--;
			} else {
				LOG_WRN("High priority report count %s", is_increment ? "overflow" : "underflow");
			}
			break;
		}

		case PACKET_PRIORITY_MEDIUM:
		{
			if (med_prio_report_count < UINT16_MAX && is_increment) {
				med_prio_report_count++;
			} else if (!is_increment && med_prio_report_count > 0) {
				med_prio_report_count--;
			} else {
				LOG_WRN("Medium priority report count %s", is_increment ? "overflow" : "underflow");
			}
			break;
		}

		case PACKET_PRIORITY_LOW:
		{
			if (low_prio_report_count < UINT16_MAX && is_increment) {
				low_prio_report_count++;
			} else if (!is_increment && low_prio_report_count > 0) {
				low_prio_report_count--;
			} else {
				LOG_WRN("Low priority report count %s", is_increment ? "overflow" : "underflow");
			}
			break;
		}

		default:
			LOG_WRN("Invalid priority %d for %s report count", priority, is_increment ? "incrementing" : "decrementing");
	}

	LOG_INF_YEL("Report Counts (%s) - High: %d, Medium: %d, Low: %d", is_increment ? "Incrementing" : "Decrementing", high_prio_report_count, med_prio_report_count, low_prio_report_count);
	return;
}

static void report_slot_free(int idx, uint8_t sender_idx)
{
	update_report_count(report_slot[idx].priority, false);
	if (sender_idx >= 0 && sender_idx < MAX_ANCHORS) {
		free_sender_slot(sender_idx);
	}
	report_slot[idx].active = false;
	report_slot[idx].is_sent = false;
	report_slot[idx].upstream_ready = false;
	report_slot[idx].is_transfered = false;
	nbtimeout_stop(&report_slot[idx].idle_timeout);
}

static uint8_t chunk_size_for(uint8_t chunk_idx, uint8_t sender_idx)
{
	if (chunk_idx == report_sender[sender_idx].chunk_count - 1) {
		return report_sender[sender_idx].last_chunk_size;
	}
	return SEND_DATA_MAX;
}

static int send_next_chunk(uint16_t dst_id, uint8_t dst_type, uint8_t sender_idx)
{
	uint8_t idx = report_sender[sender_idx].next_chunk;
	if (idx >= report_sender[sender_idx].chunk_count) {
		LOG_ERR("send_next_chunk called but all chunks already sent");
		return -EINVAL;
	}
	uint8_t csz = chunk_size_for(idx, sender_idx);

	memset(&chunk_pkt, 0, sizeof(chunk_pkt));
	chunk_pkt.gen_device_id = report_sender[sender_idx].gen_device_id;
	chunk_pkt.data_id = report_sender[sender_idx].data_id;
	chunk_pkt.chunk_index = idx;

	uint32_t addr = slot_psram_addr(0) + (uint32_t)idx * SEND_DATA_MAX;
	int err = psram_read(addr, chunk_pkt.data, csz);
	if (err) {
		LOG_ERR("psram_read @0x%06x failed (%d), aborting transfer", addr, err);
		report_sender[sender_idx].active = false;
		return err;
	}

	send_report_chunk(&chunk_pkt, dst_id, dst_type, report_sender[sender_idx].priority, sender_idx);
	report_sender[sender_idx].next_chunk = idx + 1;
	return 0;
}

static uint8_t validate_slot(const report_init_t *pkt)
{
	bool already_active = false;
	int idx = find_slot(pkt->gen_device_id, pkt->data_id);
	if (idx >= 0) {
		already_active = true;
	} else if (idx < 0) {
		idx = alloc_slot();
		if (idx < 0) {
			LOG_WRN("REPORT_INIT rejected: no free slot");
			return STATUS_RESOURCE_UNAVAILABLE;
		}
	}

	if (report_slot[idx].upstream_ready && already_active) {
		LOG_WRN("REPORT_INIT rejected: slot already active and ready for upstream for gen %d prio %d", pkt->gen_device_id, pkt->hdr.priority);
		return STATUS_ALREADY_EXISTS;
	} else if (already_active) {
		LOG_WRN("REPORT_INIT warning: slot already active but not ready for upstream, reusing slot for gen %d prio %d", pkt->gen_device_id, pkt->hdr.priority);
	} else {
		update_report_count(pkt->hdr.priority, true);
	}

	report_slot[idx].active = true;
	report_slot[idx].gen_device_id = pkt->gen_device_id;
	report_slot[idx].data_id = pkt->data_id;
	report_slot[idx].report_time_ms = pkt->report_time_ms;
	report_slot[idx].priority = pkt->hdr.priority;
	report_slot[idx].total_size = pkt->total_size;
	report_slot[idx].chunk_count = pkt->chunk_count;
	report_slot[idx].last_chunk_size = pkt->last_chunk_size;
	report_slot[idx].crc32 = pkt->crc32;
	memset(report_slot[idx].received, 0, sizeof(report_slot[idx].received));
	report_slot[idx].received_count = 0;

	nbtimeout_init(&report_slot[idx].idle_timeout, DATA_SLOT_TIMEOUT_MS, 0);
	nbtimeout_start(&report_slot[idx].idle_timeout);

	LOG_INF("REPORT_INIT accepted (alloc_slot %d) for gen %d data_id %d, total_size %d, chunk_count %d, priority %d", idx, pkt->gen_device_id, pkt->data_id, pkt->total_size, pkt->chunk_count, pkt->hdr.priority);

	return STATUS_SUCCESS;
}

static uint8_t validate_report_chunk(const report_chunk_t *pkt)
{
	int idx = find_slot(pkt->gen_device_id, pkt->data_id);
	if (idx < 0) {
		LOG_WRN("REPORT_CHUNK rejected: no active slot for gen %d data_id %d", pkt->gen_device_id, pkt->data_id);
		return STATUS_NOT_FOUND;
	}

	if (pkt->chunk_index >= report_slot[idx].chunk_count) {
		LOG_WRN("REPORT_CHUNK rejected: invalid chunk index %d (total chunks %d)", pkt->chunk_index, report_slot[idx].chunk_count);
		return STATUS_INVALID_PARAMETER;
	}

	if (report_slot[idx].received[pkt->chunk_index]) {
		LOG_WRN("REPORT_CHUNK rejected: chunk index %d already received for gen %d data_id %d", pkt->chunk_index, pkt->gen_device_id, pkt->data_id);
		return STATUS_ALREADY_EXISTS;
	}

	bool already_recieved = report_slot[idx].received[pkt->chunk_index];

	if(!already_recieved) {
		uint8_t csz = (pkt->chunk_index == report_slot[idx].chunk_count - 1) ? report_slot[idx].last_chunk_size : SEND_DATA_MAX;
		uint32_t addr = slot_psram_addr(idx) + (uint32_t)pkt->chunk_index * SEND_DATA_MAX;
		int err = psram_write(addr, pkt->data, csz);
		if (err) {
			LOG_ERR("psram_write @0x%06x failed (%d), aborting transfer", addr, err);
			return STATUS_FAILURE;
		}
		report_slot[idx].received[pkt->chunk_index] = true;
		report_slot[idx].received_count++;
	}

	nbtimeout_start(&report_slot[idx].idle_timeout);

	if (report_slot[idx].received_count == report_slot[idx].chunk_count) {
		// All chunks received, verify CRC
		uint32_t base_addr = slot_psram_addr(idx);
		uint32_t crc = 0;
		uint32_t bytes_remaining = report_slot[idx].total_size;
		uint16_t offset = 0;
		bool first_stage = true;

		while (bytes_remaining > 0) {
			uint16_t n = (bytes_remaining > CRC_VERIFY_STAGE_SIZE) ? CRC_VERIFY_STAGE_SIZE : bytes_remaining;
			int err = psram_read(base_addr + offset, crc_stage, n);
			if (err) {
				LOG_ERR("Transfer complete but psram_read @0x%06x failed (%d), freeing slot", base_addr + offset, err);
				report_slot_free(idx, -1);
				return STATUS_CRC_FAIL;
			}
			crc = first_stage ? crc32_ieee(crc_stage, n) : crc32_ieee_update(crc, crc_stage, n);
			first_stage = false;
			offset += n;
			bytes_remaining -= n;
		}

		if (crc == report_slot[idx].crc32) {
			LOG_INF("CRC match for gen %d data_id %d, transfer complete", pkt->gen_device_id, pkt->data_id);
			report_slot[idx].upstream_ready = true;
			nbtimeout_stop(&report_slot[idx].idle_timeout);
			if (get_device_type() == DEVICE_TYPE_GATEWAY) {
				// Start duplicate report detection timeout for gateway
				nbtimeout_init(&report_slot[idx].idle_timeout, DATA_DUP_TIMEOUT_MS, 0);
				nbtimeout_start(&report_slot[idx].idle_timeout);
			}
		} else {
			LOG_ERR("CRC mismatch for gen %d data_id %d: expected 0x%08x computed 0x%08x, freeing slot", pkt->gen_device_id, pkt->data_id, report_slot[idx].crc32, crc);
			report_slot_free(idx, -1);
			return STATUS_CRC_FAIL;
		}
	}
	return STATUS_SUCCESS;
}

static void gateway_report_tick(void)
{
	if (get_device_type() != DEVICE_TYPE_GATEWAY) {
		return;
	}

	for (int i = 0; i < DATA_SLOT_COUNT; i++) {
		if (report_slot[i].active && report_slot[i].upstream_ready && report_slot[i].is_sent && nbtimeout_expired(&report_slot[i].idle_timeout)) {
			LOG_WRN("Report gen %d data_id %d idle timeout expired, freeing slot", report_slot[i].gen_device_id, report_slot[i].data_id);
			report_slot_free(i, -1);
		}
	}

	static const char HEX_LUT[] = "0123456789ABCDEF";

	for (int idx = 0; idx < DATA_SLOT_COUNT; idx++) {
		if (report_slot[idx].active && report_slot[idx].upstream_ready && !report_slot[idx].is_sent) {
			LOG_INF("Attempting to upstream report for gen %d data_id %d", report_slot[idx].gen_device_id, report_slot[idx].data_id);

			// Find Serial Number first
			int sn_idx = -1;
			for (sn_idx = 0; sn_idx < mesh_count; sn_idx++) {
				if (mesh_devices[sn_idx].device_id == report_slot[idx].gen_device_id) {
					break;
				}
				if (sn_idx == mesh_count - 1) {
					LOG_ERR("Failed to find mesh device for gen_device_id %d, cannot process slot", report_slot[idx].gen_device_id);
					continue;
				}
			}

			uint8_t payload[MAX_REPORT_SIZE];
			uint32_t addr = slot_psram_addr(idx);
			int err = psram_read(addr, payload, report_slot[idx].total_size);
			if (err) {
				LOG_ERR("psram_read @0x%06x failed (%d), cannot process slot", addr, err);
				continue;
			}

			// AT#REPORT="<sn>","<data_id>","<timestamp>","<total_size>","<crc32>","<hex_payload>"
			char cmd[SLM_UART_AT_COMMAND_LEN];
			int n = snprintf(cmd, sizeof(cmd), "\r\nAT#REPORT=\"%016llX\",\"%04X\",\"%016llx\",\"%04X\",\"%08lX\",\"",
					(unsigned long long)mesh_devices[sn_idx].serial_num, report_slot[idx].data_id, (unsigned long long)report_slot[idx].report_time_ms, (unsigned)report_slot[idx].total_size,
					(unsigned long)report_slot[idx].crc32);

			if (n < 0 || (size_t)n >= sizeof(cmd)) {
			LOG_ERR("snprintf overflow building AT#REPORT header");
			continue;
			}

			size_t need = (size_t)n + (size_t)report_slot[idx].total_size * 2 + 3;
			if (need >= sizeof(cmd)) {
				LOG_ERR("AT#REPORT line too long (%u bytes needed)", (unsigned)need);
				continue;
			}
			for (uint16_t b = 0; b < report_slot[idx].total_size; b++) {
				cmd[n++] = HEX_LUT[(payload[b] >> 4) & 0x0F];
				cmd[n++] = HEX_LUT[payload[b] & 0x0F];
			}
			cmd[n++] = '"';
			cmd[n++] = '\r';
			cmd[n++] = '\n';

			int tx_err = slm_at_tx_write((const uint8_t *)cmd, (size_t)n, false);
			if (tx_err) {
				LOG_ERR("slm_at_tx_write failed (%d) for slot %d", tx_err, idx);
				continue;
			}

			LOG_INF("AT#REPORT emitted for slot %d, marking as sent", idx);
			report_slot[idx].is_sent = true;
			return;
		}
	}
}

static void anchor_report_tick(void)
{
	if (get_device_type() != DEVICE_TYPE_ANCHOR) {
		return;
	}

	for (int sender_idx = 0; sender_idx < MAX_ANCHORS; sender_idx++) {

		int idx = 0;

		if (report_sender[sender_idx].active) {
			if (nbtimeout_expired(&report_sender[sender_idx].timeout)) {
				LOG_WRN("Sender timeout expired for gen %d data_id %d, marking sender as inactive to retry", report_sender[sender_idx].gen_device_id, report_sender[sender_idx].data_id);
				idx = find_slot(report_sender[sender_idx].gen_device_id, report_sender[sender_idx].data_id);
				if (idx >= 0) {
					report_slot[idx].is_sent = false;
				} else {
					LOG_ERR("Sender timeout but failed to find matching report slot for gen %d data_id %d", report_sender[sender_idx].gen_device_id, report_sender[sender_idx].data_id);
				}
				nbtimeout_stop(&report_sender[sender_idx].timeout);
				report_sender[sender_idx].active = false;
			}
			return;
		}

		if (high_prio_report_count > 0) {
			for (idx = 0; idx < DATA_SLOT_COUNT; idx++) {
				if (report_slot[idx].active && report_slot[idx].upstream_ready && !report_slot[idx].is_sent && report_slot[idx].priority == PACKET_PRIORITY_HIGH) {
					break;
				}
			}
		} else if (med_prio_report_count > 0) {
			for (idx = 0; idx < DATA_SLOT_COUNT; idx++) {
				if (report_slot[idx].active && report_slot[idx].upstream_ready && !report_slot[idx].is_sent && report_slot[idx].priority == PACKET_PRIORITY_MEDIUM) {
					break;
				}
			}
		} else if (low_prio_report_count > 0) {
			for (idx = 0; idx < DATA_SLOT_COUNT; idx++) {
				if (report_slot[idx].active && report_slot[idx].upstream_ready && !report_slot[idx].is_sent && report_slot[idx].priority == PACKET_PRIORITY_LOW) {
					break;
				}
			}
		} else {
			return;
		}

		if (idx >= DATA_SLOT_COUNT) {
			return;
		}

		// Validate report details before attempting
		if (report_slot[idx].gen_device_id == 0 || report_slot[idx].data_id == 0 || report_slot[idx].total_size == 0 || report_slot[idx].chunk_count == 0) {
			LOG_WRN("Invalid report details in slot %d, gen_device_id %d data_id %d prio %d", idx, report_slot[idx].gen_device_id, report_slot[idx].data_id, report_slot[idx].priority);
			update_report_count(report_slot[idx].priority, false);
			return;
		}

		LOG_INF("Attempting to resend report for gen %d data_id %d", report_slot[idx].gen_device_id, report_slot[idx].data_id);

		report_sender[sender_idx].active = true;
		report_sender[sender_idx].dst_id = infra_devices[0].entry.device_id;
		report_sender[sender_idx].gen_device_id = report_slot[idx].gen_device_id;
		report_sender[sender_idx].data_id = report_slot[idx].data_id;
		report_sender[sender_idx].priority = report_slot[idx].priority;
		report_sender[sender_idx].total_size = report_slot[idx].total_size;
		report_sender[sender_idx].chunk_count = report_slot[idx].chunk_count;
		report_sender[sender_idx].last_chunk_size = report_slot[idx].last_chunk_size;
		report_sender[sender_idx].crc32 = report_slot[idx].crc32;
		report_sender[sender_idx].next_chunk = 0;

		report_init_t init_pkt = {
			.gen_device_id = report_sender[sender_idx].gen_device_id,
			.data_id = report_sender[sender_idx].data_id,
			.report_time_ms = k_uptime_get(),
			.total_size = report_sender[sender_idx].total_size,
			.chunk_count = report_sender[sender_idx].chunk_count,
			.last_chunk_size = report_sender[sender_idx].last_chunk_size,
			.crc32 = report_sender[sender_idx].crc32,
		};
		LOG_INF_CYAN("Sender is active and processing report for gen %d report_id %d", report_sender[sender_idx].gen_device_id, report_sender[sender_idx].data_id);
		send_report_init(&init_pkt, report_sender[sender_idx].dst_id, infra_devices[0].entry.device_type, report_sender[sender_idx].priority);
		nbtimeout_init(&report_sender[sender_idx].timeout, SENDER_TIMEOUT_MS, 0);
		nbtimeout_start(&report_sender[sender_idx].timeout);
		report_slot[idx].is_sent = true;
	}
}

static void sensor_report_tick(void)
{
	if (get_device_type() != DEVICE_TYPE_SENSOR) {
		return;
	}

	for (int sender_idx = 0; sender_idx < MAX_ANCHORS; sender_idx++) {

		int idx = 0;

		if (report_sender[sender_idx].active) {
			if (nbtimeout_expired(&report_sender[sender_idx].timeout)) {
				LOG_WRN("Sender timeout expired for gen %d data_id %d, marking sender as inactive to retry", report_sender[sender_idx].gen_device_id, report_sender[sender_idx].data_id);
				idx = find_slot(report_sender[sender_idx].gen_device_id, report_sender[sender_idx].data_id);
				if (idx >= 0) {
					report_slot[idx].is_sent = false;
				} else {
					LOG_ERR("Sender timeout but failed to find matching report slot for gen %d data_id %d", report_sender[sender_idx].gen_device_id, report_sender[sender_idx].data_id);
				}
				nbtimeout_stop(&report_sender[sender_idx].timeout);
				report_sender[sender_idx].active = false;
			}
			return;
		}

		if (high_prio_report_count > 0) {
			for (idx = 0; idx < DATA_SLOT_COUNT; idx++) {
				if (report_slot[idx].active && report_slot[idx].upstream_ready && !report_slot[idx].is_sent && report_slot[idx].priority == PACKET_PRIORITY_HIGH) {
					break;
				}
			}
		} else if (med_prio_report_count > 0) {
			for (idx = 0; idx < DATA_SLOT_COUNT; idx++) {
				if (report_slot[idx].active && report_slot[idx].upstream_ready && !report_slot[idx].is_sent && report_slot[idx].priority == PACKET_PRIORITY_MEDIUM) {
					break;
				}
			}
		} else if (low_prio_report_count > 0) {
			for (idx = 0; idx < DATA_SLOT_COUNT; idx++) {
				if (report_slot[idx].active && report_slot[idx].upstream_ready && !report_slot[idx].is_sent && report_slot[idx].priority == PACKET_PRIORITY_LOW) {
					break;
				}
			}
		} else {
			return;
		}

		if (idx >= DATA_SLOT_COUNT) {
			return;
		}

		// Validate report details before attempting
		if (report_slot[idx].gen_device_id == 0 || report_slot[idx].data_id == 0 || report_slot[idx].total_size == 0 || report_slot[idx].chunk_count == 0) {
			LOG_WRN("Invalid report details in slot %d, gen_device_id %d data_id %d prio %d", idx, report_slot[idx].gen_device_id, report_slot[idx].data_id, report_slot[idx].priority);
			update_report_count(report_slot[idx].priority, false);
			return;
		}

		LOG_INF("Attempting to resend report for gen %d data_id %d (Slot %d)", report_slot[idx].gen_device_id, report_slot[idx].data_id, idx);

		report_sender[sender_idx].active = true;
		report_sender[sender_idx].dst_id = infra_devices[0].entry.device_id;
		report_sender[sender_idx].gen_device_id = report_slot[idx].gen_device_id;
		report_sender[sender_idx].data_id = report_slot[idx].data_id;
		report_sender[sender_idx].priority = report_slot[idx].priority;
		report_sender[sender_idx].total_size = report_slot[idx].total_size;
		report_sender[sender_idx].chunk_count = report_slot[idx].chunk_count;
		report_sender[sender_idx].last_chunk_size = report_slot[idx].last_chunk_size;
		report_sender[sender_idx].crc32 = report_slot[idx].crc32;
		report_sender[sender_idx].next_chunk = 0;

		report_init_t init_pkt = {
			.gen_device_id = report_sender[sender_idx].gen_device_id,
			.data_id = report_sender[sender_idx].data_id,
			.report_time_ms = k_uptime_get(),
			.total_size = report_sender[sender_idx].total_size,
			.chunk_count = report_sender[sender_idx].chunk_count,
			.last_chunk_size = report_sender[sender_idx].last_chunk_size,
			.crc32 = report_sender[sender_idx].crc32,
		};
		LOG_INF_CYAN("Sender %d is active and processing report for gen %d report_id %d", sender_idx, report_sender[sender_idx].gen_device_id, report_sender[sender_idx].data_id);
		send_report_init(&init_pkt, report_sender[sender_idx].dst_id, infra_devices[0].entry.device_type, report_sender[sender_idx].priority);
		nbtimeout_init(&report_sender[sender_idx].timeout, SENDER_TIMEOUT_MS, 0);
		nbtimeout_start(&report_sender[sender_idx].timeout);
		report_slot[idx].is_sent = true;
	}
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

	LOG_INF_GRN("----> Sending REPORT_INIT to device %s ID:%d for SENSOR ID:%d (Report ID: %d)", device_type_str(dst_type), dst_id, pkt->gen_device_id, pkt->data_id);
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_report_init_ack(report_init_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id)
{
	pkt->hdr.packet_type = PACKET_REPORT_INIT_ACK;
	pkt->hdr.device_type = get_device_type();
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracking_id;
	pkt->hdr.device_id = dst_id;

	LOG_INF_GRN("----> Sending REPORT_INIT_ACK to device %s ID:%d for SENSOR ID:%d (Report ID: %d)", device_type_str(dst_type), dst_id, pkt->gen_device_id, pkt->data_id);
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_report_chunk(report_chunk_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t sender_idx)
{
	pkt->hdr.packet_type = PACKET_REPORT_CHUNK;
	pkt->hdr.device_type = get_device_type();
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracker_next_id();
	pkt->hdr.device_id = dst_id;

	// Add tracker entry for retries
	tracker_add(dst_id, get_device_id(), pkt->hdr.tracking_id, PACKET_REPORT_CHUNK, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, pkt, sizeof(*pkt));

	LOG_INF_GRN("----> Sending REPORT_CHUNK to device %s ID:%d for SENSOR ID:%d (Report ID: %d, Chunk: %d, Size: %d)", device_type_str(dst_type), dst_id, pkt->gen_device_id, pkt->data_id, pkt->chunk_index, chunk_size_for(pkt->chunk_index, sender_idx));
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_report_chunk_ack(report_chunk_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id)
{
	pkt->hdr.packet_type = PACKET_REPORT_CHUNK_ACK;
	pkt->hdr.device_type = get_device_type();
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracking_id;
	pkt->hdr.device_id = dst_id;

	LOG_INF_GRN("----> Sending REPORT_CHUNK_ACK to device %s ID:%d for SENSOR ID:%d (Report ID: %d, Chunk: %d)", device_type_str(dst_type), dst_id, pkt->gen_device_id, pkt->data_id, pkt->chunk_index);
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

	LOG_INF_GRN("----> Sending REPORT_RECEIVED to device %s ID:%d for SENSOR ID:%d (Report ID: %d)", device_type_str(dst_type), dst_id, pkt->gen_device_id, pkt->data_id);
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_report_received_ack(report_received_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id)
{
	pkt->hdr.packet_type = PACKET_REPORT_RECEIVED_ACK;
	pkt->hdr.device_type = get_device_type();
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracking_id;
	pkt->hdr.device_id = dst_id;

	LOG_INF_GRN("----> Sending REPORT_RECEIVED_ACK to device %s ID:%d for SENSOR ID:%d (Report ID: %d)", device_type_str(dst_type), dst_id, pkt->gen_device_id, pkt->data_id);
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

	LOG_INF_MAG("Received REPORT_INIT from %s ID:%d for SENSOR ID:%d (Report ID: %d) prio %d size %u chunks %u crc 0x%08x", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id, pkt->data_id, pkt->hdr.priority, pkt->total_size, pkt->chunk_count, pkt->crc32);

	report_init_ack_t ack = {
		.gen_device_id = pkt->gen_device_id,
		.data_id = pkt->data_id,
	};
	
	if (pkt->total_size == 0 || pkt->total_size > MAX_REPORT_SIZE || pkt->chunk_count == 0 || pkt->chunk_count > DATA_MAX_CHUNKS || pkt->last_chunk_size == 0 || pkt->last_chunk_size > SEND_DATA_MAX) {
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
	
	LOG_INF_MAG("Received REPORT_INIT_ACK from %s ID:%d for SENSOR ID:%d (Report ID: %d) prio %d status 0x%02x", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id, pkt->data_id, pkt->hdr.priority, pkt->hdr.status);

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
				uint8_t sender_idx = find_sender_slot(pkt->gen_device_id, pkt->data_id);
				if (sender_idx < 0) {
					LOG_ERR("No active sender found for gen %d data_id %d, this should not happen", pkt->gen_device_id, pkt->data_id);
					return;
				}
				if (pkt->hdr.status == STATUS_SUCCESS) {
					// Start sending data chunks if status is success
					if (!report_sender[sender_idx].active || report_sender[sender_idx].dst_id != dst_id || report_sender[sender_idx].gen_device_id != pkt->gen_device_id || report_sender[sender_idx].data_id != pkt->data_id) {
						LOG_WRN("REPORT_INIT_ACK from %d but sender %d inactive or dst mismatch", dst_id, sender_idx);
						return;
					}
					send_next_chunk(dst_id, pkt->hdr.device_type, sender_idx);
					return;
				} else if (pkt->hdr.status == STATUS_ALREADY_EXISTS) {
					LOG_WRN("Received REPORT_INIT_ACK with status ALREADY_EXISTS for gen %d data_id %d", pkt->gen_device_id, pkt->data_id);
					int idx = find_slot(pkt->gen_device_id, pkt->data_id);
					if (idx < 0) {
						LOG_ERR("No active slot found for ALREADY_EXISTS ack for gen %d data_id %d, this should not happen", pkt->gen_device_id, pkt->data_id);
					} else {
						report_slot_free(idx, sender_idx);
					}
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

	LOG_INF_MAG("Received REPORT_CHUNK from %s ID:%d for SENSOR ID:%d (Report ID: %d) prio %d chunk %d", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id, pkt->data_id, pkt->hdr.priority, pkt->chunk_index);

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

	LOG_INF_MAG("Received REPORT_CHUNK_ACK from %s ID:%d for SENSOR ID:%d (Report ID: %d) prio %d chunk %d status 0x%02x", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id, pkt->data_id, pkt->hdr.priority, pkt->chunk_index, pkt->hdr.status);

	// Remove tracker
	tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

	// Validate that the ack is for the current active sender transfer, if not ignore the packet
	uint8_t sender_idx = find_sender_slot(pkt->gen_device_id, pkt->data_id);
	if (sender_idx < 0) {
		LOG_WRN("No active sender found for gen %d data_id %d, ignoring REPORT_CHUNK_ACK", pkt->gen_device_id, pkt->data_id);
		return;
	}
	if (!report_sender[sender_idx].active || report_sender[sender_idx].gen_device_id != pkt->gen_device_id || report_sender[sender_idx].data_id != pkt->data_id || report_sender[sender_idx].dst_id != dst_id) {
		LOG_WRN("REPORT_CHUNK_ACK ignored: sender inactive or gen/data/dst mismatch (got %d/%d/%d, have %d/%d/%d)", pkt->gen_device_id, pkt->data_id, dst_id, report_sender[sender_idx].gen_device_id, report_sender[sender_idx].data_id, report_sender[sender_idx].dst_id);
		return;
	}

	if (report_sender[sender_idx].next_chunk >= report_sender[sender_idx].chunk_count && pkt->hdr.status == STATUS_SUCCESS) {
		LOG_INF("Transfer complete: %u bytes in %u chunks to ID:%d", report_sender[sender_idx].total_size, report_sender[sender_idx].chunk_count, report_sender[sender_idx].dst_id);
		int idx = find_slot(report_sender[sender_idx].gen_device_id, report_sender[sender_idx].data_id);
		if (idx < 0) {
			LOG_ERR("No active slot found for completed transfer gen %d data_id %d, this should not happen", report_sender[sender_idx].gen_device_id, report_sender[sender_idx].data_id);
		} else {
			report_slot_free(idx, sender_idx);
		}
		report_sender[sender_idx].active = false;
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
					report_slot[pkt->chunk_index].is_sent = false;
					report_sender[sender_idx].active = false;
					return;
				} else if (pkt->hdr.status == STATUS_FAILURE || pkt->hdr.status == STATUS_INVALID_PARAMETER) {
					// Rebuild same chunk and resend
					report_sender[sender_idx].next_chunk--;
				}
				// Send next chunk if status is success
				if (send_next_chunk(dst_id, pkt->hdr.device_type, sender_idx) != 0) {
					report_slot_free(find_slot(report_sender[sender_idx].gen_device_id, report_sender[sender_idx].data_id), sender_idx);
					report_sender[sender_idx].active = false;
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

	LOG_INF_MAG("Received REPORT_RECEIVED from %s ID:%d for SENSOR ID:%d (Report ID: %d) prio %d", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id, pkt->data_id, pkt->hdr.priority);

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
				if ((pkt->route_info[1].device_id == 0xFFFF || pkt->route_info[1].device_id == 0) && pkt->route_info[0].device_id == get_device_id()) {
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
				} else if (pkt->route_info[1].device_id != 0xFFFF || pkt->route_info[1].device_id != 0) {
					// Forward report received packet to next hop
					report_received_t recv_pkt;
					memcpy(&recv_pkt, pkt, sizeof(report_received_t));
					for (int i = 0; i < MAX_DEPTH - 1; i++) {
						recv_pkt.route_info[i] = recv_pkt.route_info[i + 1];
					}
					recv_pkt.route_info[MAX_DEPTH - 1].device_id = 0xFFFF;
					recv_pkt.route_info[MAX_DEPTH - 1].hop_num = 0xFF;
					ack.hdr.status = STATUS_SUCCESS;
					send_report_received(&recv_pkt, recv_pkt.route_info[0].device_id, DEVICE_TYPE_ANCHOR);
				} else {
					LOG_WRN("Invalid route info in REPORT_RECEIVED, rejecting");
					ack.hdr.status = STATUS_INVALID_PARAMETER;
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

	LOG_INF_MAG("Received REPORT_RECEIVED_ACK from %s ID:%d for SENSOR ID:%d (Report ID: %d) prio %d status 0x%02x", device_type_str(pkt->hdr.device_type), dst_id, pkt->gen_device_id, pkt->data_id, pkt->hdr.priority, pkt->hdr.status);

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

int report_init(void)
{
	for (int i = 0; i < DATA_SLOT_COUNT; i++) {
		report_slot[i].active = false;
	}
	for (int i = 0; i < MAX_ANCHORS; i++) {
		report_sender[i].active = false;
	}

	LOG_INF("Report module Initialized with %d slots (slot size=%d) at PSRAM 0x%06x-0x%06x", DATA_SLOT_COUNT,
		MAX_REPORT_SIZE, DATA_PSRAM_BASE, DATA_PSRAM_BASE + DATA_PSRAM_SIZE - 1);

	return 0;
}

void report_tick(void)
{
	for (int i = 0; i < DATA_SLOT_COUNT; i++) {
		if (report_slot[i].active && nbtimeout_expired(&report_slot[i].idle_timeout)) {
			LOG_WRN("Slot %d (gen %d) idle timeout, freeing (%u/%u chunks)",
				i, report_slot[i].gen_device_id,
				report_slot[i].received_count, report_slot[i].chunk_count);
			report_slot_free(i, -1);
		}
	}

	switch (get_device_type()) {
		case DEVICE_TYPE_GATEWAY:
			gateway_report_tick();
			break;

		case DEVICE_TYPE_ANCHOR:
			anchor_report_tick();
			break;

		case DEVICE_TYPE_SENSOR:
			sensor_report_tick();
			break;

		default:
			break;
	}
}

int validate_at_report(const slm_at_structure_t *report, uint8_t priority, const uint8_t *data)
{
	if (report == NULL || data == NULL) {
		return -EINVAL;
	}
	if (report->data_len == 0 || report->data_len > MAX_REPORT_SIZE) {
		return -EINVAL;
	}

	/* Find an existing slot for this device, otherwise allocate a fresh one. */
	int idx = find_slot(get_device_id(), report->data_id);
	if (idx < 0) {
		idx = alloc_slot();
		if (idx < 0) {
			return -ENOMEM;
		}
	}

	report_slot[idx].active = true;
	report_slot[idx].upstream_ready = true;
	report_slot[idx].gen_device_id = get_device_id();
	report_slot[idx].data_id = report->data_id;
	report_slot[idx].priority  = priority;
	report_slot[idx].total_size = report->data_len;
	report_slot[idx].chunk_count = (report->data_len + SEND_DATA_MAX - 1) / SEND_DATA_MAX;
	report_slot[idx].last_chunk_size = (report->data_len % SEND_DATA_MAX) ? (report->data_len % SEND_DATA_MAX) : SEND_DATA_MAX;
	report_slot[idx].crc32 = report->data_crc32;
	report_slot[idx].received_count = report_slot[idx].chunk_count;
	report_slot[idx].received[0] = true;
	report_slot[idx].received[1] = true;

	update_report_count(priority, true);

	int err = psram_write(slot_psram_addr(idx), data, report->data_len);
	if (err) {
		LOG_ERR("psram_write @0x%06x failed (%d), freeing slot", slot_psram_addr(idx), err);
		report_slot_free(idx, -1);
		return err;
	}

	LOG_INF_YEL("Data ID: %d, Priority: %d and Allocated slot %d", report->data_id, priority, idx);
	LOG_INF_YEL("High prio count: %d, med prio count: %d, low prio count: %d ", high_prio_report_count, med_prio_report_count, low_prio_report_count);

	return 0;
}

int report_slot_release_by_id(uint16_t device_id, uint16_t report_id, bool is_success)
{
	if (device_id == 0xFFFF) {
		LOG_WRN("Invalid device_id 0xFFFF for releasing report slot");
		return -EINVAL;
	}

	for (int i = 0; i < DATA_SLOT_COUNT; i++) {
		if (report_slot[i].gen_device_id != device_id) {
			continue;
		}
		if (report_slot[i].active && report_slot[i].is_sent && report_slot[i].data_id == report_id) {
			if (is_success) {
				uint16_t hop_num = find_hop_num(report_slot[i].gen_device_id, DEVICE_TYPE_SENSOR);
				if (hop_num == 0xFF) {
					LOG_ERR("No route to gen_device_id %d, cannot send REPORT_RECEIVED", report_slot[i].gen_device_id);
					return -ENOENT;
				} else if (hop_num == 0) {
					LOG_INF("Sensor ID:%d is directly connected to this gateway, sending REPORT_RECEIVED without route discovery", report_slot[i].gen_device_id);
					// Build report received packet and send directly without route discovery
					report_received_t recv_pkt = {
						.gen_device_id = report_slot[i].gen_device_id,
						.data_id = report_slot[i].data_id,
						.hdr.status = STATUS_SUCCESS,
					};
					send_report_received(&recv_pkt, report_slot[i].gen_device_id, DEVICE_TYPE_SENSOR);
					return 0;
				}
				route_discovery_t rd_pkt = {
					.device_id = report_slot[i].gen_device_id,
					.device_type = DEVICE_TYPE_SENSOR,
					.hop_num = hop_num,
					.data_id = i,
					.data_type = DATA_TYPE_REPORT,
				};
				for (int i = 0; i < infra_count; i++) {
					send_route_discovery(&rd_pkt, infra_devices[i].entry.device_id, infra_devices[i].entry.device_type, STATUS_SUCCESS);
				}
				return 0;
			} else {
				LOG_WRN("Releasing slot %d for report ID:%d after failed processing", i, report_id);
				report_slot[i].is_sent = false;
				uint8_t sender_idx = find_sender_slot(report_slot[i].gen_device_id, report_slot[i].data_id);
				if (sender_idx >= 0) {
					report_sender[sender_idx].active = false;
				} else {
					LOG_WRN("No active sender found for report ID:%d to release", report_id);
				}
				return 0;
			}
		}
	}
	LOG_WRN("No active slot found for report ID:%d to release", report_id);
	return -ENOENT;
}