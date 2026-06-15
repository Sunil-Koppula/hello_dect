#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/byteorder.h>
#include "large_data.h"
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

LOG_MODULE_REGISTER(large_data, CONFIG_LARGE_DATA_LOG_LEVEL);

static uint8_t crc_stage[LD_CRC_VERIFY_STAGE_SIZE];

struct large_data_sender ld_sender[4]; // support up to 4 concurrent sending transfers
bool is_ld_slot_empty[LARGE_DATA_SLOT_COUNT];    /* Track empty slots */
struct large_data_slot ld_slot[LARGE_DATA_RECEIVER_SLOT_COUNT];

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------**** Large Data Sender ****--------------------------------------------------------------------------- */
static int find_sender_slot(uint16_t dst_id, uint8_t data_id)
{
	for (int i = 0; i < 4; i++) {
		if (ld_sender[i].active && ld_sender[i].gen_device_id == dst_id && ld_sender[i].data_id == data_id) {
			return i;
		}
	}
	return -1;
}

static void free_sender_slot(int idx)
{
	ld_sender[idx].active = false;
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------**** Large Data Slots ****---------------------------------------------------------------------------- */

static int find_large_data_slot(uint16_t gen_device_id, uint8_t data_id)
{
    for (int i = 0; i < LARGE_DATA_RECEIVER_SLOT_COUNT; i++) {
        if (ld_slot[i].active && ld_slot[i].gen_device_id == gen_device_id && ld_slot[i].data_id == data_id) {
            return i;
        }
    }
    return -1;
}

static int alloc_large_data_slot(uint32_t size)
{
    int idx;
    for (idx = 0; idx < LARGE_DATA_RECEIVER_SLOT_COUNT; idx++) {
        if (!ld_slot[idx].active) {
            break;
        }
    }
    if (idx >= LARGE_DATA_RECEIVER_SLOT_COUNT) {
        LOG_WRN("alloc_large_data_slot: no free receiver slot");
        return -1;
    }
    uint16_t available_slots = 0;
    for (int i = 0; i < LARGE_DATA_SLOT_COUNT; i++) {
        if (is_ld_slot_empty[i]) {
            available_slots++;
            if (available_slots * SEND_DATA_MAX >= size) {
                // Mark these slots as used
                for (int j = i - available_slots + 1; j <= i; j++) {
                    is_ld_slot_empty[j] = false;
                }
                ld_slot[idx].active = true;
                ld_slot[idx].base_addr = LARGE_DATA_PSRAM_BASE + (i - available_slots + 1) * SEND_DATA_MAX;
                LOG_INF("Allocated large data slot %d (PSRAM 0x%06x-0x%06x) for size %u bytes", idx, ld_slot[idx].base_addr, ld_slot[idx].base_addr + available_slots * SEND_DATA_MAX - 1, size);
                return idx;
            }
        } else {
            available_slots = 0;
        }
    }
    return -1;
}

static void free_large_data_slot(int idx, uint8_t sender_idx)
{
    ld_slot[idx].active = false;
    nbtimeout_stop(&ld_slot[idx].idle_timeout);
    // Mark the corresponding PSRAM slots as empty
    uint16_t start_slot = (ld_slot[idx].base_addr - LARGE_DATA_PSRAM_BASE) / SEND_DATA_MAX;
    uint16_t slot_count = (ld_slot[idx].total_size + SEND_DATA_MAX - 1) / SEND_DATA_MAX;
    for (int i = start_slot; i < start_slot + slot_count; i++) {
        is_ld_slot_empty[i] = true;
    }
    // Free the corresponding sender slot
    if (sender_idx < 4 && sender_idx >= 0) {
        free_sender_slot(sender_idx);
    }
}

static uint16_t chunk_size_for_large_data(uint16_t chunk_idx, uint8_t sender_idx)
{
    uint16_t last_chunk_index = ld_sender[sender_idx].total_chunks - 1;
    if (chunk_idx == last_chunk_index) {
        return ld_sender[sender_idx].last_chunk_size;
    }
    return SEND_DATA_MAX;
}

static int send_next_large_data_chunk(uint16_t dst_id, uint8_t dst_type, uint8_t sender_idx)
{
    uint16_t page_idx = ld_sender[sender_idx].next_chunk / 20;
    uint8_t chunk_idx = ld_sender[sender_idx].next_chunk % 20;
    uint16_t csz = chunk_size_for_large_data(ld_sender[sender_idx].next_chunk, sender_idx);

    large_data_chunk_t chunk_pkt;
    memset(&chunk_pkt, 0, sizeof(chunk_pkt));
    chunk_pkt.gen_device_id = ld_sender[sender_idx].gen_device_id;
    chunk_pkt.data_id = ld_sender[sender_idx].data_id;
    chunk_pkt.page_index = page_idx;
    chunk_pkt.chunk_index = chunk_idx;

    uint32_t addr = ld_sender[sender_idx].base_addr + (uint32_t)ld_sender[sender_idx].next_chunk * SEND_DATA_MAX;
    int err = psram_read(addr, chunk_pkt.data, csz);
    if (err) {
        LOG_ERR("psram_read @0x%06x failed (%d), aborting transfer", addr, err);
        ld_sender[sender_idx].active = false;
        return err;
    }

    send_large_data_chunk(&chunk_pkt, dst_id, dst_type, ld_sender[sender_idx].priority);
    ld_sender[sender_idx].next_chunk++;
    return 0;
}

static uint8_t validate_large_data_slot(const large_data_init_t *pkt)
{
    int idx = find_large_data_slot(pkt->gen_device_id, pkt->data_id);
    if (idx < 0) {
        idx = alloc_large_data_slot(pkt->total_size);
        if (idx < 0) {
            LOG_WRN("LARGE_DATA_INIT rejected: no free slot for gen %d data_id %d", pkt->gen_device_id, pkt->data_id);
            return STATUS_RESOURCE_UNAVAILABLE;
        }

        ld_slot[idx].active = true;
        ld_slot[idx].upstream_ready = false;
        ld_slot[idx].gen_device_id = pkt->gen_device_id;
        ld_slot[idx].data_id = pkt->data_id;
        ld_slot[idx].priority = pkt->hdr.priority;
        ld_slot[idx].total_size = pkt->total_size;
        ld_slot[idx].page_count = pkt->page_count;
        ld_slot[idx].last_chunk_size = pkt->last_chunk_size;
        ld_slot[idx].crc32 = pkt->crc32;
        ld_slot[idx].received_count = 0;
        ld_slot[idx].total_chunks = (pkt->total_size + SEND_DATA_MAX - 1) / SEND_DATA_MAX;
        nbtimeout_init(&ld_slot[idx].idle_timeout, LARGE_DATA_SLOT_TIMEOUT_MS, 0);
    } else if (ld_slot[idx].active && ld_slot[idx].upstream_ready) {
        return STATUS_COMPLETE;
    } else if (ld_slot[idx].active) {
        return STATUS_ALREADY_EXISTS;
    }

    nbtimeout_start(&ld_slot[idx].idle_timeout);

    return STATUS_SUCCESS;
}

static int large_data_verify_crc32(int idx)
{
    uint32_t crc = 0;
    uint32_t bytes_remaining = ld_slot[idx].total_size;
    uint32_t offset = 0;
    bool first_stage = true;
    LOG_INF("Starting CRC verification for gen %d data_id %d, total_size %u bytes", ld_slot[idx].gen_device_id, ld_slot[idx].data_id, ld_slot[idx].total_size);

    while (bytes_remaining > 0) {
        uint16_t n = (bytes_remaining > LD_CRC_VERIFY_STAGE_SIZE) ? LD_CRC_VERIFY_STAGE_SIZE : bytes_remaining;
        int err = psram_read(ld_slot[idx].base_addr + offset, crc_stage, n);
        if (err) {
            LOG_ERR("CRC verify: psram_read @0x%06x failed (%d)", ld_slot[idx].base_addr + offset, err);
            return err;
        }
        crc = first_stage ? crc32_ieee(crc_stage, n) : crc32_ieee_update(crc, crc_stage, n);
        first_stage = false;
        offset += n;
        bytes_remaining -= n;
    }

    if (crc != ld_slot[idx].crc32) {
        LOG_ERR("CRC mismatch for gen %d data_id %d: expected 0x%08x computed 0x%08x",
                ld_slot[idx].gen_device_id, ld_slot[idx].data_id, ld_slot[idx].crc32, crc);
        return STATUS_CRC_FAIL;
    }
    return 0;
}

static uint8_t validate_large_data_chunk(const large_data_chunk_t *pkt)
{
    int idx = find_large_data_slot(pkt->gen_device_id, pkt->data_id);
    if (idx < 0) {
        LOG_WRN("LARGE_DATA_CHUNK rejected: no active slot for gen %d data_id %d", pkt->gen_device_id, pkt->data_id);
        return STATUS_NOT_FOUND;
    }

    if (pkt->page_index >= ld_slot[idx].page_count || pkt->chunk_index > 20 || (pkt->page_index * 20 + pkt->chunk_index) > ld_slot[idx].total_chunks) {
        LOG_WRN("LARGE_DATA_CHUNK rejected: Page: %d Chunk: %d out of bounds for gen %d data_id %d (page_count: %d, total_chunks: %d, last_chunk_size: %d)", pkt->page_index, pkt->chunk_index, pkt->gen_device_id, pkt->data_id, ld_slot[idx].page_count, ld_slot[idx].total_chunks, ld_slot[idx].last_chunk_size);
        return STATUS_INVALID_PARAMETER;
    }

    if (ld_slot[idx].received_count > (pkt->page_index * 20 + pkt->chunk_index)) {
        LOG_WRN("LARGE_DATA_CHUNK rejected: Page: %d Chunk: %d already received for gen %d data_id %d", pkt->page_index, pkt->chunk_index, pkt->gen_device_id, pkt->data_id);
        return STATUS_ALREADY_EXISTS;
    }

    uint16_t linear = pkt->page_index * 20 + pkt->chunk_index;
    uint32_t addr = ld_slot[idx].base_addr + (uint32_t)linear * SEND_DATA_MAX;
    uint16_t csz = (linear == ld_slot[idx].total_chunks - 1) ? ld_slot[idx].last_chunk_size : SEND_DATA_MAX;

    int err = psram_write(addr, pkt->data, csz);
    LOG_INF("Addr: 0x%06x", addr);
    if (err) {
        LOG_ERR("psram_write @0x%06x failed (%d), aborting transfer", addr, err);
        return STATUS_FAILURE;
    }

    ld_slot[idx].received_count++;
    nbtimeout_start(&ld_slot[idx].idle_timeout);

    // Validate CRC and check all chunks received when received_count matches total_chunks, to handle out-of-order chunk reception
    if (ld_slot[idx].received_count >= ld_slot[idx].total_chunks) {
        int err = large_data_verify_crc32(idx);
        if (err != 0) {
            LOG_ERR("CRC verification failed for gen %d data_id %d after receiving all chunks, aborting transfer", ld_slot[idx].gen_device_id, ld_slot[idx].data_id);
            free_large_data_slot(idx, -1);
            return STATUS_CRC_FAIL;
        }
    }
    return STATUS_SUCCESS;
}

static void gateway_large_data_tick(void)
{
    if (get_device_type() != DEVICE_TYPE_GATEWAY) {
        return;
    }

    for (int i = 0; i < LARGE_DATA_RECEIVER_SLOT_COUNT; i++) {
        if (ld_slot[i].active && nbtimeout_expired(&ld_slot[i].idle_timeout)) {
            LOG_WRN("Large data slot for gen %d data_id %d timed out, freeing slot", ld_slot[i].gen_device_id, ld_slot[i].data_id);
            free_large_data_slot(i, -1);
        }
    }

    // Implement UART communication later

    return;
}

static void anchor_large_data_tick(void)
{
    if (get_device_type() != DEVICE_TYPE_ANCHOR) {
        return;
    }

    for (int sender_idx = 0; sender_idx < 4; sender_idx++) {
        int idx = 0;

        if (ld_sender[sender_idx].active) {
            if (nbtimeout_expired(&ld_sender[sender_idx].timeout)) {
                LOG_WRN("Sender timeout expired for gen %d data_id %d, marking sender as inactive to retry", ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id);
                idx = find_large_data_slot(ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id);
                if (idx >= 0) {
                    ld_slot[idx].is_sent = false;
                } else {
                    LOG_ERR("Sender timeout but failed to find matching report slot for gen %d data_id %d", ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id);
                }
                nbtimeout_stop(&ld_sender[sender_idx].timeout);
                ld_sender[sender_idx].active = false;
            }
            continue;
        }

        for (idx = 0; idx < LARGE_DATA_RECEIVER_SLOT_COUNT; idx++) {
            if (ld_slot[idx].active && ld_slot[idx].upstream_ready && !ld_slot[idx].is_sent) {
                break;
            }
        }

        if (idx >= LARGE_DATA_RECEIVER_SLOT_COUNT) {
            return;
        }

        // Validate large data slot before sending/attempting
        if (ld_slot[idx].gen_device_id == 0 || ld_slot[idx].data_id == 0 || ld_slot[idx].total_size == 0
            || ld_slot[idx].page_count == 0 || ld_slot[idx].total_chunks == 0 || ld_slot[idx].last_chunk_size == 0
            || ld_slot[idx].crc32 == 0 || ld_slot[idx].base_addr == 0) {
            LOG_WRN("Invalid large data slot details in slot %d, gen_device_id %d data_id %d", idx, ld_slot[idx].gen_device_id, ld_slot[idx].data_id);
            free_large_data_slot(idx, -1);
            return;
        }

        LOG_INF("Starting to send large data for gen %d data_id %d to gateway, total_size %u bytes, total_chunks %d", ld_slot[idx].gen_device_id, ld_slot[idx].data_id, ld_slot[idx].total_size, ld_slot[idx].total_chunks);

        // Initialize sender slot
        ld_sender[sender_idx].active = true;
        ld_sender[sender_idx].dst_id = infra_devices[0].entry.device_id;
        ld_sender[sender_idx].gen_device_id = ld_slot[idx].gen_device_id;
        ld_sender[sender_idx].data_id = ld_slot[idx].data_id;
        ld_sender[sender_idx].priority = ld_slot[idx].priority;
        ld_sender[sender_idx].total_size = ld_slot[idx].total_size;
        ld_sender[sender_idx].page_count = ld_slot[idx].page_count;
        ld_sender[sender_idx].last_chunk_size = ld_slot[idx].last_chunk_size;
        ld_sender[sender_idx].total_chunks = ld_slot[idx].total_chunks;
        ld_sender[sender_idx].next_chunk = 0;
        ld_sender[sender_idx].crc32 = ld_slot[idx].crc32;
        ld_sender[sender_idx].base_addr = ld_slot[idx].base_addr;

        large_data_init_t init_pkt = {
            .gen_device_id = ld_sender[sender_idx].gen_device_id,
            .data_id = ld_sender[sender_idx].data_id,
            .report_time_ms = k_uptime_get(),
            .total_size = ld_sender[sender_idx].total_size,
            .page_count = ld_sender[sender_idx].page_count,
            .last_chunk_size = ld_sender[sender_idx].last_chunk_size,
            .crc32 = ld_sender[sender_idx].crc32,
        };
        LOG_INF("Sender slot %d initialized for gen %d data_id %d, total_size %u bytes, total_chunks %d, page_count %d, last_chunk_size %d", sender_idx, ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id, ld_sender[sender_idx].total_size, ld_sender[sender_idx].total_chunks, ld_sender[sender_idx].page_count, ld_sender[sender_idx].last_chunk_size);
        send_large_data_init(&init_pkt, ld_sender[sender_idx].dst_id, infra_devices[0].entry.device_type, ld_sender[sender_idx].priority);
        nbtimeout_init(&ld_sender[sender_idx].timeout, LD_SENDER_TIMEOUT_MS, 0);
        nbtimeout_start(&ld_sender[sender_idx].timeout);
        ld_slot[idx].is_sent = true;
    }

    return;
}

static void sensor_large_data_tick(void)
{
    if (get_device_type() != DEVICE_TYPE_SENSOR) {
        return;
    }

    for (int sender_idx = 0; sender_idx < 4; sender_idx++) {
        int idx = 0;

        if (ld_sender[sender_idx].active) {
            if (nbtimeout_expired(&ld_sender[sender_idx].timeout)) {
                LOG_WRN("Sender timeout expired for gen %d data_id %d, marking sender as inactive to retry", ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id);
                idx = find_large_data_slot(ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id);
                if (idx >= 0) {
                    // ld_slot[idx].is_sent = false;
                } else {
                    LOG_ERR("Sender timeout but failed to find matching report slot for gen %d data_id %d", ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id);
                }
                nbtimeout_stop(&ld_sender[sender_idx].timeout);
                ld_sender[sender_idx].active = false;
            }
            continue;
        }

        for (idx = 0; idx < LARGE_DATA_RECEIVER_SLOT_COUNT; idx++) {
            if (ld_slot[idx].active && ld_slot[idx].upstream_ready && !ld_slot[idx].is_sent) {
                break;
            }
        }

        if (idx >= LARGE_DATA_RECEIVER_SLOT_COUNT) {
            return;
        }

        // Validate large data slot before sending/attempting
        if (ld_slot[idx].gen_device_id == 0 || ld_slot[idx].data_id == 0 || ld_slot[idx].total_size == 0
            || ld_slot[idx].page_count == 0 || ld_slot[idx].total_chunks == 0 || ld_slot[idx].last_chunk_size == 0
            || ld_slot[idx].crc32 == 0 || ld_slot[idx].base_addr == 0) {
            LOG_WRN("Invalid large data slot details in slot %d, gen_device_id %d data_id %d", idx, ld_slot[idx].gen_device_id, ld_slot[idx].data_id);
            free_large_data_slot(idx, -1);
            return;
        }

        LOG_INF("Starting to send large data for gen %d data_id %d to gateway, total_size %u bytes, total_chunks %d", ld_slot[idx].gen_device_id, ld_slot[idx].data_id, ld_slot[idx].total_size, ld_slot[idx].total_chunks);

        // Initialize sender slot
        ld_sender[sender_idx].active = true;
        ld_sender[sender_idx].dst_id = infra_devices[0].entry.device_id;
        ld_sender[sender_idx].gen_device_id = ld_slot[idx].gen_device_id;
        ld_sender[sender_idx].data_id = ld_slot[idx].data_id;
        ld_sender[sender_idx].priority = ld_slot[idx].priority;
        ld_sender[sender_idx].total_size = ld_slot[idx].total_size;
        ld_sender[sender_idx].page_count = ld_slot[idx].page_count;
        ld_sender[sender_idx].last_chunk_size = ld_slot[idx].last_chunk_size;
        ld_sender[sender_idx].total_chunks = ld_slot[idx].total_chunks;
        ld_sender[sender_idx].next_chunk = 0;
        ld_sender[sender_idx].crc32 = ld_slot[idx].crc32;
        ld_sender[sender_idx].base_addr = ld_slot[idx].base_addr;

        large_data_init_t init_pkt = {
            .gen_device_id = ld_sender[sender_idx].gen_device_id,
            .data_id = ld_sender[sender_idx].data_id,
            .report_time_ms = k_uptime_get(),
            .total_size = ld_sender[sender_idx].total_size,
            .page_count = ld_sender[sender_idx].page_count,
            .last_chunk_size = ld_sender[sender_idx].last_chunk_size,
            .crc32 = ld_sender[sender_idx].crc32,
        };
        LOG_INF("Sender slot %d initialized for gen %d data_id %d, total_size %u bytes, total_chunks %d, page_count %d, last_chunk_size %d", sender_idx, ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id, ld_sender[sender_idx].total_size, ld_sender[sender_idx].total_chunks, ld_sender[sender_idx].page_count, ld_sender[sender_idx].last_chunk_size);
        send_large_data_init(&init_pkt, ld_sender[sender_idx].dst_id, infra_devices[0].entry.device_type, ld_sender[sender_idx].priority);
        nbtimeout_init(&ld_sender[sender_idx].timeout, LD_SENDER_TIMEOUT_MS, 0);
        nbtimeout_start(&ld_sender[sender_idx].timeout);
        ld_slot[idx].is_sent = true;
    }

    return;
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------------**** TX Helpers ****------------------------------------------------------------------------------- */

int send_large_data_init(large_data_init_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority)
{
    pkt->hdr.packet_type = PACKET_LARGE_DATA_INIT;
    pkt->hdr.device_type = get_device_type();
    pkt->hdr.priority = priority;
    pkt->hdr.tracking_id = tracker_next_id();
    pkt->hdr.device_id = dst_id;

    // Add tracker entry for retries
    tracker_add(dst_id, get_device_id(), pkt->hdr.tracking_id, PACKET_LARGE_DATA_INIT, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, pkt, sizeof(*pkt));

    LOG_INF("----> Sending LARGE_DATA_INIT to device %s ID:%d for DATA ID:%d", device_type_str(dst_type), dst_id, pkt->data_id);
    return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_large_data_init_ack(large_data_init_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id)
{
    pkt->hdr.packet_type = PACKET_LARGE_DATA_INIT_ACK;
    pkt->hdr.device_type = get_device_type();
    pkt->hdr.priority = priority;
    pkt->hdr.tracking_id = tracking_id;
    pkt->hdr.device_id = dst_id;

    LOG_INF("----> Sending LARGE_DATA_INIT_ACK to device %s ID:%d for DATA ID:%d", device_type_str(dst_type), dst_id, pkt->data_id);
    return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_large_data_chunk(large_data_chunk_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority)
{
    pkt->hdr.packet_type = PACKET_LARGE_DATA_CHUNK;
    pkt->hdr.device_type = get_device_type();
    pkt->hdr.priority = priority;
    pkt->hdr.tracking_id = tracker_next_id();
    pkt->hdr.device_id = dst_id;

    // Add tracker entry for retries
    tracker_add(dst_id, get_device_id(), pkt->hdr.tracking_id, PACKET_LARGE_DATA_CHUNK, PACKET_LARGE_DATA_TIMEOUT_MS, PACKET_MAX_RETRIES, pkt, sizeof(*pkt));

    LOG_INF("----> Sending LARGE_DATA_CHUNK to device %s ID:%d for DATA ID:%d (Page: %d Chunk: %d, Size: %d)", device_type_str(dst_type), dst_id, pkt->data_id, pkt->page_index, pkt->chunk_index, sizeof(pkt->data));
    return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_large_data_chunk_ack(large_data_chunk_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id)
{
    pkt->hdr.packet_type = PACKET_LARGE_DATA_CHUNK_ACK;
    pkt->hdr.device_type = get_device_type();
    pkt->hdr.priority = priority;
    pkt->hdr.tracking_id = tracking_id;
    pkt->hdr.device_id = dst_id;

    LOG_INF("----> Sending LARGE_DATA_CHUNK_ACK to device %s ID:%d for DATA ID:%d (Page: %d Chunk: %d)", device_type_str(dst_type), dst_id, pkt->data_id, pkt->page_index, pkt->chunk_index);
    return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_large_data_received(large_data_received_t *pkt, uint16_t dst_id, uint8_t dst_type)
{
    pkt->hdr.packet_type = PACKET_LARGE_DATA_RECEIVED;
    pkt->hdr.device_type = get_device_type();
    pkt->hdr.priority = PACKET_PRIORITY_HIGH;
    pkt->hdr.tracking_id = tracker_next_id();
    pkt->hdr.device_id = dst_id;

    LOG_INF("----> Sending LARGE_DATA_RECEIVED to device %s ID:%d for DATA ID:%d", device_type_str(dst_type), dst_id, pkt->data_id);
    return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_large_data_received_ack(large_data_received_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id)
{
    pkt->hdr.packet_type = PACKET_LARGE_DATA_RECEIVED_ACK;
    pkt->hdr.device_type = get_device_type();
    pkt->hdr.priority = priority;
    pkt->hdr.tracking_id = tracking_id;
    pkt->hdr.device_id = dst_id;

    LOG_INF("----> Sending LARGE_DATA_RECEIVED_ACK to device %s ID:%d for DATA ID:%d", device_type_str(dst_type), dst_id, pkt->data_id);
    return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------**** Handlers Functions ****--------------------------------------------------------------------------- */

void handle_large_data_init(const large_data_init_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in LARGE_DATA_INIT from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    LOG_INF("Received LARGE_DATA_INIT from %s ID:%d for DATA ID:%d total_size %d page_count %d last_chunk_size %d", device_type_str(pkt->hdr.device_type), dst_id, pkt->data_id, pkt->total_size, pkt->page_count, pkt->last_chunk_size);

   large_data_init_ack_t ack = {
    .gen_device_id = pkt->gen_device_id,
    .data_id = pkt->data_id,
   };

   if (pkt->total_size == 0 || pkt->total_size > LARGE_DATA_MAX_TRANSFER_SIZE || pkt->page_count == 0 || pkt->last_chunk_size == 0 || pkt->last_chunk_size > LARGE_DATA_SLOT_SIZE) {   
    LOG_WRN("DATA_INIT rejected: invalid size or page count for gen %d data_id %d (total_size: %d, page_count: %d, last_chunk_size: %d)", pkt->gen_device_id, pkt->data_id, pkt->total_size, pkt->page_count, pkt->last_chunk_size);
        ack.hdr.status = STATUS_INVALID_PARAMETER;
        send_large_data_init_ack(&ack, dst_id, pkt->hdr.device_type, PACKET_PRIORITY_HIGH, pkt->hdr.tracking_id);
        return;
    }

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Check there is already an active slot or free slot for this incoming data init , if not reject the packet
                uint8_t status = validate_large_data_slot(pkt);
                ack.hdr.status = status;
            } else {
                // Reject large data init except from anchor and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            // Sensor's will not process or transfer any large data, so reject any incoming large data init 
			return;
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any large data init if this device has invalid type
            return;
        }
        break;
    }

    send_large_data_init_ack(&ack, dst_id, pkt->hdr.device_type, PACKET_PRIORITY_HIGH, pkt->hdr.tracking_id);
}

void handle_large_data_init_ack(const large_data_init_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in LARGE_DATA_INIT_ACK from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    LOG_INF("Received LARGE_DATA_INIT_ACK from %s ID:%d for DATA ID:%d status 0x%02x", device_type_str(pkt->hdr.device_type), dst_id, pkt->data_id, pkt->hdr.status);

    // Remove tracker
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway will never receive large data init ack because only anchor and sensor can receive data init ack, so just ignore if received
            return;
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        case DEVICE_TYPE_SENSOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                uint8_t sender_idx = find_sender_slot(pkt->gen_device_id, pkt->data_id);
                if (sender_idx < 0) {
                    LOG_WRN("LARGE_DATA_INIT_ACK from %d for DATA ID:%d has no matching sender state, rejecting", dst_id, pkt->data_id);
                    return;
                }
                if (pkt->hdr.status == STATUS_SUCCESS || pkt->hdr.status == STATUS_ALREADY_EXISTS) {
                    // Start sending large data chunks if ack is success
                    if (!ld_sender[sender_idx].active || ld_sender[sender_idx].dst_id != dst_id || ld_sender[sender_idx].gen_device_id != pkt->gen_device_id || ld_sender[sender_idx].data_id != pkt->data_id) {
                        LOG_WRN("LARGE_DATA_INIT_ACK from %d but sender inactive or dst mismatch", dst_id);
                        return;
                    }
                    send_next_large_data_chunk(dst_id, pkt->hdr.device_type, sender_idx);
                } else if (pkt->hdr.status == STATUS_COMPLETE) {
                    // This means the data has already been transferred and stored in receiver side, so sender can consider this transfer complete and clean up the sender state
                    LOG_INF("Received LARGE_DATA_INIT_ACK with COMPLETE status from %d, marking transfer complete", dst_id);
                    free_sender_slot(sender_idx);
                    int idx = find_large_data_slot(ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id);
                    if (idx < 0) {
                        LOG_WRN("LARGE_DATA_INIT_ACK with COMPLETE status but failed to find matching report slot for gen %d data_id %d", ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id);
                        return;
                    }
                    ld_slot[idx].is_transfered = true;
                }
            } else {
                // Reject large data init ack except from gateway and anchor
                return;
            }
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any large data init ack if this device has invalid type
			return;
        }
        break; 
    }

    return;
}

void handle_large_data_chunk(const large_data_chunk_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in LARGE_DATA_CHUNK from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    LOG_INF("Received LARGE_DATA_CHUNK from %s ID:%d for DATA ID:%d page %d chunk %d size %d", device_type_str(pkt->hdr.device_type), dst_id, pkt->data_id, pkt->page_index, pkt->chunk_index, sizeof(pkt->data));

    large_data_chunk_ack_t ack = {
        .gen_device_id = pkt->gen_device_id,
        .data_id = pkt->data_id,
        .page_index = pkt->page_index,
        .chunk_index = pkt->chunk_index,
    };

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                // Check the incoming chunk is valid for this device, if not reject the packet
                uint8_t status = validate_large_data_chunk(pkt);
                ack.hdr.status = status;
            } else {
                // Reject large data chunk except from anchor and sensor
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            // Sensor's will not process or transfer any large data, so reject any incoming large data chunk
            return;
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any large data chunk if this device has invalid type
            return;
        }
        break;
    }

    send_large_data_chunk_ack(&ack, dst_id, pkt->hdr.device_type, pkt->hdr.priority, pkt->hdr.tracking_id);
    return;
}

void handle_large_data_chunk_ack(const large_data_chunk_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in LARGE_DATA_CHUNK_ACK from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    LOG_INF("Received LARGE_DATA_CHUNK_ACK from %s ID:%d for DATA ID:%d page %d chunk %d status 0x%02x", device_type_str(pkt->hdr.device_type), dst_id, pkt->data_id, pkt->page_index, pkt->chunk_index, pkt->hdr.status);

    // Remove tracker
    tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

    // Validate that the ack is for the current active sender transfer, if not ignore the packet
    uint8_t sender_idx = find_sender_slot(pkt->gen_device_id, pkt->data_id);
    if (sender_idx < 0) {
        LOG_WRN("LARGE_DATA_CHUNK_ACK from %d has no matching sender state, rejecting", dst_id);
        return;
    }

    if (!ld_sender[sender_idx].active || ld_sender[sender_idx].dst_id != dst_id || ld_sender[sender_idx].gen_device_id != pkt->gen_device_id || ld_sender[sender_idx].data_id != pkt->data_id) {
        LOG_WRN("LARGE_DATA_CHUNK_ACK from %d but sender inactive or dst/gen/data mismatch", dst_id);
        return;
    }

    // Check transfer is complete or not
    if (pkt->hdr.status == STATUS_SUCCESS && pkt->page_index == (ld_sender[sender_idx].page_count - 1) && pkt->chunk_index == (ld_sender[sender_idx].total_chunks - 1) % 20) {
        LOG_INF("Large data transfer complete for gen %d data_id %d (status 0x%02x last_chunk_size %d page_count %d/%d chunk_index %d)", pkt->gen_device_id, pkt->data_id, pkt->hdr.status, ld_sender[sender_idx].last_chunk_size, ld_sender[sender_idx].page_count, pkt->page_index, pkt->chunk_index);
        free_sender_slot(sender_idx);
        int idx = find_large_data_slot(ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id);
        if (idx < 0) {
            LOG_ERR("Failed to find matching large data slot for gen %d data_id %d to free after transfer complete", ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id);
            return;
        }
        free_large_data_slot(idx, sender_idx);
        return;
    }

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway will never receive large data chunk ack because only anchor and sensor can receive data chunk ack, so just ignore if received
            return;
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        case DEVICE_TYPE_SENSOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                // If status is not found, resend the data init
                if (pkt->hdr.status == STATUS_NOT_FOUND || pkt->hdr.status == STATUS_RESOURCE_UNAVAILABLE || pkt->hdr.status == STATUS_CRC_FAIL) {
                    LOG_WRN("Received LARGE_DATA_CHUNK_ACK with NOT_FOUND, RESOURCE_UNAVAILABLE, or CRC_FAIL, resending LARGE_DATA_INIT for gen %d data_id %d", pkt->gen_device_id, pkt->data_id);
                    free_sender_slot(sender_idx);
                    int idx = find_large_data_slot(ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id);
                    if (idx < 0) {
                        LOG_ERR("Failed to find matching large data slot for gen %d data_id %d to resend LARGE_DATA_INIT", ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id);
                        return;
                    }
                    // ld_slot[idx].is_sent = false;
                    return;
                } else if (pkt->hdr.status == STATUS_FAILURE || pkt->hdr.status == STATUS_INVALID_PARAMETER) {
                    // Rebuild same chunk and resend
                    ld_sender[sender_idx].next_chunk--;
                }
                if (send_next_large_data_chunk(dst_id, pkt->hdr.device_type, sender_idx) != 0) {
                    free_sender_slot(sender_idx);
                    int idx = find_large_data_slot(ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id);
                    if (idx < 0) {
                        LOG_ERR("Failed to find matching large data slot for gen %d data_id %d to resend LARGE_DATA_INIT after chunk ack failure", ld_sender[sender_idx].gen_device_id, ld_sender[sender_idx].data_id);
                        return;
                    }
                    ld_slot[idx].is_sent = false;
                }
            } else {
                // Reject large data chunk ack except from gateway and anchor
                return;
            }
        }
        break;

         default:
        {
            // There are only 3 valid device types, reject any large data chunk ack if this device has invalid type
            return;
        }
        break;
    }

    return;
}

void handle_large_data_received(const large_data_received_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != get_device_id()) {
        return;
    }

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in LARGE_DATA_RECEIVED from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    LOG_INF("Received LARGE_DATA_RECEIVED from %s ID:%d for DATA ID:%d", device_type_str(pkt->hdr.device_type), dst_id, pkt->data_id);

    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway will never receive large data received because only anchor can receive data received, so just ignore if received
            return;
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                return;
            } else {
                // Reject large data received except from gateway and anchor
                return;
            }
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
                LOG_INF("Gateway Received the Large Data");
                return;
            } else {
                // Reject large data received except from anchor and sensor
                return;
            }
        }
        break;

        default:
        {
            // There are only 3 valid device types, reject any large data received if this device has invalid type
            return;
        }
        break;
    }

    return;
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------**** Module Init / Tick ****--------------------------------------------------------------------------- */

int large_data_init(void)
{
   for (int i = 0; i < LARGE_DATA_RECEIVER_SLOT_COUNT; i++) {
        ld_slot[i].active = false;
    }

    for (int i = 0; i < LARGE_DATA_SLOT_COUNT; i++) {
        is_ld_slot_empty[i] = true;
    }

    LOG_INF("Large Data module initialized with %d slots at PSRAM 0x%06x-0x%06x", LARGE_DATA_RECEIVER_SLOT_COUNT, LARGE_DATA_PSRAM_BASE, LARGE_DATA_PSRAM_BASE + LARGE_DATA_PSRAM_SIZE - 1);

    return 0;
}

void large_data_tick(void)
{
    switch (get_device_type()) {
        case DEVICE_TYPE_GATEWAY:
            gateway_large_data_tick();
            break;

        case DEVICE_TYPE_ANCHOR:
            anchor_large_data_tick();
            break;

        case DEVICE_TYPE_SENSOR:
            sensor_large_data_tick();
            break;

        default:
            break;
    }
}


void cmd_ld_init(const char *args)
{
    if (get_device_type() != DEVICE_TYPE_SENSOR) {
        at_error();
        return;
    }

    if (get_connected_device_id() == 0xFFFF) {
        LOG_WRN("AT#LDINIT received but no paired device");
        at_error();
        return;
    }

    uint64_t v_sn, v_id, v_total, v_chunks, v_last, v_crc32, v_crc16;
    if (field_hex_u64(args, 0, &v_sn)     != 0 ||
        field_hex_u64(args, 1, &v_id)     != 0 ||
        field_hex_u64(args, 2, &v_total)  != 0 ||
        field_hex_u64(args, 3, &v_chunks) != 0 ||
        field_hex_u64(args, 4, &v_last)   != 0 ||
        field_hex_u64(args, 5, &v_crc32)  != 0 ||
        field_hex_u64(args, 6, &v_crc16)  != 0) {
        at_error();
        return;
    }

    /* CRC16 covers the init metadata, including the whole-data CRC32, so a
     * corrupted init line — or a corrupted CRC32 — is caught on arrival. */
    uint8_t crc_buf[22];
    sys_put_be64(v_sn, &crc_buf[0]);
    sys_put_be16((uint16_t)v_id, &crc_buf[8]);
    sys_put_be32((uint32_t)v_total, &crc_buf[10]);
    sys_put_be16((uint16_t)v_chunks, &crc_buf[14]);
    sys_put_be16((uint16_t)v_last, &crc_buf[16]);
    sys_put_be32((uint32_t)v_crc32, &crc_buf[18]);
    uint16_t calc_crc16 = crc16(0x1021, 0xFFFF, crc_buf, sizeof(crc_buf));
    if (v_crc16 > 0xFFFF || calc_crc16 != (uint16_t)v_crc16) {
        LOG_WRN("AT#LDINIT CRC16 mismatch: got 0x%04llx, calc 0x%04x — init corrupt",
                (unsigned long long)v_crc16, calc_crc16);
        at_error();
        return;
    }

    /* SN must match this sensor. */
    if (v_sn != get_serial_number()) {
        LOG_WRN("AT#LDINIT SN mismatch: got 0x%016llx, expected 0x%016llx",
                (unsigned long long)v_sn, (unsigned long long)get_serial_number());
        at_error();
        return;
    }

    /* Range checks. */
    if (v_id > 0xFF || v_total == 0 || v_total > LARGE_DATA_MAX_TRANSFER_SIZE ||
        v_chunks == 0 || v_chunks > 0xFFFF || v_crc32 > 0xFFFFFFFF ||
        v_last == 0 || v_last > AT_DATA_PAYLOAD_MAX) {
        LOG_WRN("AT#LDINIT rejected: invalid params (id=%llu total=%llu chunks=%llu last=%llu)",
                (unsigned long long)v_id, (unsigned long long)v_total,
                (unsigned long long)v_chunks, (unsigned long long)v_last);
        at_error();
        return;
    }

    /* Consistency: total_size == (chunks-1)*AT_DATA_PAYLOAD_MAX + last_chunk_size. */
    if (v_total != (uint64_t)(v_chunks - 1) * AT_DATA_PAYLOAD_MAX + v_last) {
        LOG_WRN("AT#LDINIT rejected: size/chunk mismatch (total=%llu chunks=%llu last=%llu)",
                (unsigned long long)v_total, (unsigned long long)v_chunks,
                (unsigned long long)v_last);
        at_error();
        return;
    }

    /* One in-flight transfer per (this sensor, data_id). */
    int idx = find_large_data_slot(get_device_id(), (uint8_t)v_id);
    if (idx >= 0) {
        LOG_WRN("AT#LDINIT rejected: transfer for data_id %u already in progress", (uint16_t)v_id);
        at_error();
        return;
    }

    /* Reserve contiguous PSRAM staging for the whole blob (sets base_addr). */
    idx = alloc_large_data_slot((uint32_t)v_total);
    if (idx < 0) {
        LOG_WRN("AT#LDINIT rejected: no free slot for data_id %u", (uint16_t)v_id);
        at_error();
        return;
    }

    ld_slot[idx].upstream_ready = false;
    ld_slot[idx].is_sent = false;
    ld_slot[idx].is_transfered = false;
    ld_slot[idx].gen_device_id = get_device_id();
    ld_slot[idx].data_id = (uint8_t)v_id;
    ld_slot[idx].priority = PACKET_PRIORITY_HIGH;
    ld_slot[idx].total_size = (uint32_t)v_total;
    ld_slot[idx].total_chunks = v_chunks;
    ld_slot[idx].last_chunk_size = (uint16_t)v_last;
    ld_slot[idx].page_count = (v_chunks + LARGE_DATA_CHUNKS_PER_SIZE - 1) / LARGE_DATA_CHUNKS_PER_SIZE; // 20 chunks per page
    ld_slot[idx].crc32 = (uint32_t)v_crc32;
    ld_slot[idx].received_count = 0;
    nbtimeout_init(&ld_slot[idx].idle_timeout, LARGE_DATA_SLOT_TIMEOUT_MS, 0);
    nbtimeout_start(&ld_slot[idx].idle_timeout);

    LOG_INF("AT#LDINIT accepted: sn=0x%016llx id=%u total=%u chunks=%u last=%u crc32=0x%08x (PSRAM 0x%06x and slot %d)",
            (unsigned long long)v_sn, ld_slot[idx].data_id, ld_slot[idx].total_size,
            ld_slot[idx].total_chunks, ld_slot[idx].last_chunk_size, ld_slot[idx].crc32,
            ld_slot[idx].base_addr, idx);

    char resp[96];
    snprintf(resp, sizeof(resp), "#LDINIT:\"0x%016llx\",\"%u\",\"%u\",\"%u\"",
             (unsigned long long)v_sn, ld_slot[idx].data_id,
             ld_slot[idx].total_size, ld_slot[idx].total_chunks);
    at_send_line(resp);
    at_ok();
}

void cmd_ld_chunk(const char *args)
{
    if (get_device_type() != DEVICE_TYPE_SENSOR) {
        at_error();
        return;
    }

    uint64_t v_sn, v_id, v_page, v_chunk, v_crc16;
    if (field_hex_u64(args, 0, &v_sn)    != 0 ||
        field_hex_u64(args, 1, &v_id)    != 0 ||
        field_hex_u64(args, 2, &v_page)  != 0 ||
        field_hex_u64(args, 3, &v_chunk) != 0 ||
        field_hex_u64(args, 4, &v_crc16) != 0) {
        at_error();
        return;
    }

    /* Field 5: the chunk payload, hex-encoded (2 chars per byte). */
    size_t      hex_len;
    const char *payload_hex = field_get(args, 5, &hex_len);
    if (payload_hex == NULL || hex_len == 0) {
        at_error();
        return;
    }

    uint8_t payload[AT_DATA_PAYLOAD_MAX];
    int     decoded = hex_decode(payload_hex, hex_len, payload, sizeof(payload));
    if (decoded <= 0) {
        at_error();
        return;
    }

    /* CRC16 over the payload — catches corruption on the serial link. */
    uint16_t calc_crc16 = crc16(0x1021, 0xFFFF, payload, (size_t)decoded);
    if (v_crc16 > 0xFFFF || calc_crc16 != (uint16_t)v_crc16) {
        LOG_WRN("AT#LD CRC16 mismatch: got 0x%04llx, calc 0x%04x — chunk corrupt",
                (unsigned long long)v_crc16, calc_crc16);
        at_error();
        return;
    }

    /* SN must match this sensor. */
    if (v_sn != get_serial_number()) {
        LOG_WRN("AT#LD SN mismatch: got 0x%016llx, expected 0x%016llx",
                (unsigned long long)v_sn, (unsigned long long)get_serial_number());
        at_error();
        return;
    }

    if (v_id > 0xFF || v_page > 0xFF || v_chunk >= LARGE_DATA_CHUNKS_PER_SIZE) {
        LOG_WRN("AT#LD rejected: bad id/page/chunk (id=%llu page=%llu chunk=%llu, max %d/page)",
                (unsigned long long)v_id, (unsigned long long)v_page,
                (unsigned long long)v_chunk, LARGE_DATA_CHUNKS_PER_SIZE);
        at_error();
        return;
    }

    int idx = find_large_data_slot(get_device_id(), (uint8_t)v_id);
    if (idx < 0) {
        LOG_WRN("AT#LD rejected: no active transfer for data_id %u (missing AT#LDINIT?)",
                (uint16_t)v_id);
        at_error();
        return;
    }

    /* AT framing: AT_DATA_PAYLOAD_MAX bytes per chunk, indexed page/chunk. */
    uint16_t linear = (uint16_t)(v_page * LARGE_DATA_CHUNKS_PER_SIZE + v_chunk);
    if (linear >= ld_slot[idx].total_chunks) {
        LOG_WRN("AT#LD rejected: page %llu chunk %llu out of range (%u chunks) for data_id %u",
                (unsigned long long)v_page, (unsigned long long)v_chunk,
                ld_slot[idx].total_chunks, (uint16_t)v_id);
        at_error();
        return;
    }

    uint32_t offset = (uint32_t)linear * AT_DATA_PAYLOAD_MAX;
    uint16_t expected = (linear == ld_slot[idx].total_chunks - 1)
                        ? (uint16_t)(ld_slot[idx].total_size - offset)
                        : AT_DATA_PAYLOAD_MAX;
    if ((uint16_t)decoded != expected) {
        LOG_WRN("AT#LD rejected: chunk %u size %d, expected %u for data_id %u",
                linear, decoded, expected, (uint16_t)v_id);
        at_error();
        return;
    }

    /* Already staged (host retransmit) — ack idempotently without rewriting. */
    if (ld_slot[idx].received_count > linear) {
        LOG_INF("AT#LD chunk %u already received for data_id %u, ignoring payload but acking", linear, (uint16_t)v_id);
        at_ok();
        return;
    }

    LOG_DBG("Staging AT#LD chunk %u (page %u chunk %u) for data_id %u at PSRAM offset 0x%06x",
            linear, (uint16_t)v_page, (uint16_t)v_chunk, (uint16_t)v_id, offset);
    int err = psram_write(ld_slot[idx].base_addr + offset, payload, expected);
    if (err) {
        LOG_ERR("AT#LD psram_write @0x%06x failed (%d), freeing slot",
                ld_slot[idx].base_addr + offset, err);
        free_large_data_slot(idx, -1);
        at_error();
        return;
    }

    ld_slot[idx].received_count++;
    nbtimeout_start(&ld_slot[idx].idle_timeout);

    /* Last chunk staged — verify the whole-blob CRC32 from AT#LDINIT and hand
     * the slot to sensor_large_data_tick() via upstream_ready to forward it. */
    if (ld_slot[idx].received_count >= ld_slot[idx].total_chunks) {
        if (large_data_verify_crc32(idx) != 0) {
            free_large_data_slot(idx, -1);
            at_error();
            return;
        }
        ld_slot[idx].total_chunks = (ld_slot[idx].total_size + SEND_DATA_MAX - 1) / SEND_DATA_MAX;
        ld_slot[idx].last_chunk_size = (uint16_t)(ld_slot[idx].total_size - (ld_slot[idx].total_chunks - 1) * SEND_DATA_MAX);
        ld_slot[idx].page_count = (ld_slot[idx].total_chunks + LARGE_DATA_CHUNKS_PER_SIZE - 1) / LARGE_DATA_CHUNKS_PER_SIZE;
        ld_slot[idx].received_count = ld_slot[idx].total_chunks;
        ld_slot[idx].upstream_ready = true;
        nbtimeout_stop(&ld_slot[idx].idle_timeout);
        LOG_INF("AT#LD transfer complete for data_id %u (%u B), CRC32 OK — ready to forward",
                (uint16_t)v_id, ld_slot[idx].total_size);
    }

    at_ok();
}