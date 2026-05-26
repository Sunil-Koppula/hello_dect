#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
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

LOG_MODULE_REGISTER(large_data, CONFIG_LARGE_DATA_LOG_LEVEL);

#define CRC_VERIFY_STAGE_SIZE 1024  /* Read and process data in 1KB stages for CRC verification */
#define PACKET_LARGE_DATA_TIMEOUT_MS 100

static large_data_chunk_t chunk_pkt;
static uint8_t crc_stage[CRC_VERIFY_STAGE_SIZE];

struct large_data_sender ld_sender;

bool is_ld_slot_empty[LARGE_DATA_SLOT_COUNT];

struct large_data_slot {
    bool        active;
    bool        upstream_ready;
    uint16_t    gen_device_id;
    uint8_t     data_id;
    uint8_t     priority;
    uint32_t    total_size;
    uint8_t     page_count;
    uint16_t    last_page_size;
    uint16_t    total_chunks;
    uint32_t    crc32;
    uint16_t    received_count;
    uint32_t    base_addr;
    struct nbtimeout idle_timeout;    
};

static struct large_data_slot slots[LARGE_DATA_RECEIVER_SLOT_COUNT];

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------**** Large Data Slots ****---------------------------------------------------------------------------- */

static int find_large_data_slot(uint16_t gen_device_id, uint8_t data_id)
{
    for (int i = 0; i < LARGE_DATA_RECEIVER_SLOT_COUNT; i++) {
        if (slots[i].active && slots[i].gen_device_id == gen_device_id && slots[i].data_id == data_id) {
            return i;
        }
    }
    return -1;
}

static int alloc_large_data_slot(uint32_t size)
{
    int idx;
    for (idx = 0; idx < LARGE_DATA_RECEIVER_SLOT_COUNT; idx++) {
        if (!slots[idx].active) {
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
                slots[idx].active = true;
                slots[idx].base_addr = LARGE_DATA_PSRAM_BASE + (i - available_slots + 1) * SEND_DATA_MAX;
                LOG_INF("Allocated large data slot %d (PSRAM 0x%06x-0x%06x) for size %u bytes", idx, slots[idx].base_addr, slots[idx].base_addr + available_slots * SEND_DATA_MAX - 1, size);
                return idx;
            }
        } else {
            available_slots = 0;
        }
    }
    return -1;
}

static void free_large_data_slot(int idx)
{
    slots[idx].active = false;
    nbtimeout_stop(&slots[idx].idle_timeout);
    // Mark the corresponding PSRAM slots as empty
    uint16_t start_slot = (slots[idx].base_addr - LARGE_DATA_PSRAM_BASE) / SEND_DATA_MAX;
    uint16_t slot_count = (slots[idx].total_size + SEND_DATA_MAX - 1) / SEND_DATA_MAX;
    for (int i = start_slot; i < start_slot + slot_count; i++) {
        is_ld_slot_empty[i] = true;
    }
}

static uint8_t chunk_size_for_large_data(uint8_t chunk_idx)
{
    uint16_t last_chunk_index = (ld_sender.page_count - 1) * 20 + ((ld_sender.last_page_size + SEND_DATA_MAX - 1) / SEND_DATA_MAX);
    if (chunk_idx == last_chunk_index) {
        return (ld_sender.last_page_size % SEND_DATA_MAX == 0) ? SEND_DATA_MAX : (ld_sender.last_page_size - (ld_sender.last_page_size / SEND_DATA_MAX) * SEND_DATA_MAX);
    }
    return SEND_DATA_MAX;
}

static int send_next_large_data_chunk(uint16_t dst_id, uint8_t dst_type)
{
    uint16_t page_idx = ld_sender.next_chunk / 20;
    uint8_t chunk_idx = ld_sender.next_chunk % 20;
    uint8_t csz = chunk_size_for_large_data(ld_sender.next_chunk);

    memset(&chunk_pkt, 0, sizeof(chunk_pkt));
    chunk_pkt.gen_device_id = ld_sender.gen_device_id;
    chunk_pkt.data_id = ld_sender.data_id;
    chunk_pkt.page_index = page_idx;
    chunk_pkt.chunk_index = chunk_idx;

    uint32_t addr = ld_sender.base_addr + (uint32_t)ld_sender.next_chunk * SEND_DATA_MAX;
    int err = psram_read(addr, chunk_pkt.data, csz);
    if (err) {
        LOG_ERR("psram_read @0x%06x failed (%d), aborting transfer", addr, err);
        ld_sender.active = false;
        return err;
    }

    send_large_data_chunk(&chunk_pkt, dst_id, dst_type, ld_sender.priority);
    ld_sender.next_chunk++;
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

        slots[idx].active = true;
        slots[idx].upstream_ready = false;
        slots[idx].gen_device_id = pkt->gen_device_id;
        slots[idx].data_id = pkt->data_id;
        slots[idx].priority = pkt->hdr.priority;
        slots[idx].total_size = pkt->total_size;
        slots[idx].page_count = pkt->page_count;
        slots[idx].last_page_size = pkt->last_page_size;
        slots[idx].crc32 = pkt->crc32;
        slots[idx].received_count = 0;
        slots[idx].total_chunks = (pkt->page_count - 1) * 20 + ((pkt->last_page_size + SEND_DATA_MAX - 1) / SEND_DATA_MAX);
        nbtimeout_init(&slots[idx].idle_timeout, LARGE_DATA_SLOT_TIMEOUT_MS, 0);
    } else if (slots[idx].active && slots[idx].upstream_ready) {
        return STATUS_COMPLETE;
    } else if (slots[idx].active) {
        return STATUS_ALREADY_EXISTS;
    }

    nbtimeout_start(&slots[idx].idle_timeout);

    return STATUS_SUCCESS;
}

static uint8_t validate_large_data_chunk(const large_data_chunk_t *pkt)
{
    int idx = find_large_data_slot(pkt->gen_device_id, pkt->data_id);
    if (idx < 0) {
        LOG_WRN("LARGE_DATA_CHUNK rejected: no active slot for gen %d data_id %d", pkt->gen_device_id, pkt->data_id);
        return STATUS_NOT_FOUND;
    }

    if (pkt->page_index >= slots[idx].page_count || pkt->chunk_index > 20 || (pkt->page_index * 20 + pkt->chunk_index) > slots[idx].total_chunks) {
        LOG_WRN("LARGE_DATA_CHUNK rejected: Page: %d Chunk: %d out of bounds for gen %d data_id %d (page_count: %d, total_chunks: %d)", pkt->page_index, pkt->chunk_index, pkt->gen_device_id, pkt->data_id, slots[idx].page_count, slots[idx].total_chunks);
        return STATUS_INVALID_PARAMETER;
    }

    if (slots[idx].received_count > (pkt->page_index * 20 + pkt->chunk_index)) {
        LOG_WRN("LARGE_DATA_CHUNK rejected: Page: %d Chunk: %d already received for gen %d data_id %d", pkt->page_index, pkt->chunk_index, pkt->gen_device_id, pkt->data_id);
        return STATUS_ALREADY_EXISTS;
    }

    uint32_t addr = slots[idx].base_addr + (uint32_t)(pkt->page_index * 20 + pkt->chunk_index) * SEND_DATA_MAX;
    uint8_t csz = chunk_size_for_large_data(pkt->page_index * 20 + pkt->chunk_index);
    int err = psram_write(addr, pkt->data, csz);
    if (err) {
        LOG_ERR("psram_write @0x%06x failed (%d), aborting transfer", addr, err);
        return STATUS_FAILURE;
    }
    slots[idx].received_count++;
    nbtimeout_start(&slots[idx].idle_timeout);

    // Validate CRC and check all chunks received when received_count matches total_chunks, to handle out-of-order chunk reception
    if (slots[idx].received_count >= slots[idx].total_chunks) {
        // All chunks received, verify CRC
        uint32_t crc = 0;
        uint32_t bytes_remaining = slots[idx].total_size;
        uint16_t offset = 0;
        bool first_stage = true;

        while (bytes_remaining > 0) {
            uint16_t n = (bytes_remaining > CRC_VERIFY_STAGE_SIZE) ? CRC_VERIFY_STAGE_SIZE : bytes_remaining;
            int err = psram_read(slots[idx].base_addr + offset, crc_stage, n);
            if (err) {
                LOG_ERR("Transfer complete but psram_read @0x%06x failed (%d), freeing slot", slots[idx].base_addr + offset, err);
                free_large_data_slot(idx);
                return STATUS_FAILURE;
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
            free_large_data_slot(idx);
            return STATUS_CRC_FAIL;
        }
    }
    return STATUS_SUCCESS;
}


/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------------**** TX Helpers ****------------------------------------------------------------------------------- */

int send_large_data_init(large_data_init_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority)
{
    pkt->hdr.packet_type = PACKET_LARGE_DATA_INIT;
    pkt->hdr.device_type = DEVICE_TYPE;
    pkt->hdr.priority = priority;
    pkt->hdr.tracking_id = tracker_next_id();
    pkt->hdr.device_id = dst_id;

    // Add tracker entry for retries
    tracker_add(dst_id, radio_get_device_id(), pkt->hdr.tracking_id, PACKET_LARGE_DATA_INIT, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, pkt, sizeof(*pkt));

    LOG_INF("----> Sending LARGE_DATA_INIT to device %s ID:%d for DATA ID:%d", device_type_str(dst_type), dst_id, pkt->data_id);
    return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_large_data_init_ack(large_data_init_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id)
{
    pkt->hdr.packet_type = PACKET_LARGE_DATA_INIT_ACK;
    pkt->hdr.device_type = DEVICE_TYPE;
    pkt->hdr.priority = priority;
    pkt->hdr.tracking_id = tracking_id;
    pkt->hdr.device_id = dst_id;

    LOG_INF("----> Sending LARGE_DATA_INIT_ACK to device %s ID:%d for DATA ID:%d", device_type_str(dst_type), dst_id, pkt->data_id);
    return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_large_data_chunk(large_data_chunk_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority)
{
    pkt->hdr.packet_type = PACKET_LARGE_DATA_CHUNK;
    pkt->hdr.device_type = DEVICE_TYPE;
    pkt->hdr.priority = priority;
    pkt->hdr.tracking_id = tracker_next_id();
    pkt->hdr.device_id = dst_id;

    // Add tracker entry for retries
    tracker_add(dst_id, radio_get_device_id(), pkt->hdr.tracking_id, PACKET_LARGE_DATA_CHUNK, PACKET_LARGE_DATA_TIMEOUT_MS, PACKET_MAX_RETRIES, pkt, sizeof(*pkt));

    LOG_INF("----> Sending LARGE_DATA_CHUNK to device %s ID:%d for DATA ID:%d (Page: %d Chunk: %d, Size: %d)", device_type_str(dst_type), dst_id, pkt->data_id, pkt->page_index, pkt->chunk_index, sizeof(pkt->data));
    return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_large_data_chunk_ack(large_data_chunk_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id)
{
    pkt->hdr.packet_type = PACKET_LARGE_DATA_CHUNK_ACK;
    pkt->hdr.device_type = DEVICE_TYPE;
    pkt->hdr.priority = priority;
    pkt->hdr.tracking_id = tracking_id;
    pkt->hdr.device_id = dst_id;

    LOG_INF("----> Sending LARGE_DATA_CHUNK_ACK to device %s ID:%d for DATA ID:%d (Page: %d Chunk: %d)", device_type_str(dst_type), dst_id, pkt->data_id, pkt->page_index, pkt->chunk_index);
    return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_large_data_received(large_data_receive_t *pkt, uint16_t dst_id, uint8_t dst_type)
{
    pkt->hdr.packet_type = PACKET_LARGE_DATA_RECEIVED;
    pkt->hdr.device_type = DEVICE_TYPE;
    pkt->hdr.priority = PACKET_PRIORITY_HIGH;
    pkt->hdr.tracking_id = tracker_next_id();
    pkt->hdr.device_id = dst_id;

    LOG_INF("----> Sending LARGE_DATA_RECEIVED to device %s ID:%d for DATA ID:%d", device_type_str(dst_type), dst_id, pkt->data_id);
    return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------**** Handlers Functions ****--------------------------------------------------------------------------- */

void handle_large_data_init(const large_data_init_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in LARGE_DATA_INIT from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    LOG_INF("Received LARGE_DATA_INIT from %s ID:%d for DATA ID:%d total_size %d page_count %d", device_type_str(pkt->hdr.device_type), dst_id, pkt->data_id, pkt->total_size, pkt->page_count);

   large_data_init_ack_t ack = {
    .gen_device_id = pkt->gen_device_id,
    .data_id = pkt->data_id,
   };

   if (pkt->total_size == 0 || pkt->total_size > LARGE_DATA_MAX_TRANSFER_SIZE || pkt->page_count == 0 || pkt->last_page_size == 0 || pkt->last_page_size > LARGE_DATA_SLOT_SIZE) {   
    LOG_WRN("DATA_INIT rejected: invalid size or page count for gen %d data_id %d (total_size: %d, page_count: %d, last_page_size: %d)", pkt->gen_device_id, pkt->data_id, pkt->total_size, pkt->page_count, pkt->last_page_size);
        ack.hdr.status = STATUS_INVALID_PARAMETER;
        send_large_data_init_ack(&ack, dst_id, pkt->hdr.device_type, PACKET_PRIORITY_HIGH, pkt->hdr.tracking_id);
        return;
    }

    switch (DEVICE_TYPE) {
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
    if (pkt->hdr.device_id != radio_get_device_id()) {
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

    switch (DEVICE_TYPE) {
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
                if (pkt->hdr.status == STATUS_SUCCESS || pkt->hdr.status == STATUS_ALREADY_EXISTS) {
                    // Start sending large data chunks if ack is success
                    if (!ld_sender.active || ld_sender.dst_id != dst_id || ld_sender.gen_device_id != pkt->gen_device_id || ld_sender.data_id != pkt->data_id) {
                        LOG_WRN("LARGE_DATA_INIT_ACK from %d but sender inactive or dst mismatch", dst_id);
                        return;
                    }
                    send_next_large_data_chunk(dst_id, pkt->hdr.device_type);
                } else if (pkt->hdr.status == STATUS_COMPLETE) {
                    // This means the data has already been transferred and stored in receiver side, so sender can consider this transfer complete and clean up the sender state
                    LOG_INF("Received LARGE_DATA_INIT_ACK with COMPLETE status from %d, marking transfer complete", dst_id);
                    ld_sender.active = false;
                    free_large_data_slot(find_large_data_slot(ld_sender.gen_device_id, ld_sender.data_id));

                    large_data_receive_t recv_pkt = {
                        .gen_device_id = pkt->gen_device_id,
                        .data_id = pkt->data_id,
                    };
                    // Find dst_id from route table using gen_device_id and send large data received packet to sensor
                    // dst_id = get_next_hop_device_id(pkt->gen_device_id);
                    dst_id = 0xFFFF; // Implement later
                    if (dst_id == 0 || dst_id == 0xFFFF || dst_id == radio_get_device_id()) {
                        LOG_ERR("No route to gen_device_id %d, cannot forward LARGE_DATA_RECEIVED", pkt->gen_device_id);
                        return;
                    }
                    send_large_data_received(&recv_pkt, dst_id, DEVICE_TYPE_SENSOR);
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
    if (pkt->hdr.device_id != radio_get_device_id()) {
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

    switch (DEVICE_TYPE) {
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

    int idx = find_large_data_slot(pkt->gen_device_id, pkt->data_id);
    if (idx >= 0 && slots[idx].upstream_ready && DEVICE_TYPE == DEVICE_TYPE_GATEWAY) {
        // Notify Sensor large data received
        LOG_ERR("Need to Send Large Data Received Packet to Sensor for gen %d data_id %d", pkt->gen_device_id, pkt->data_id);
        large_data_receive_t recv_pkt = {
            .gen_device_id = pkt->gen_device_id,
            .data_id = pkt->data_id,
        };
        // Find dst_id from route table using gen_device_id and send large data received packet to sensor
        // dst_id = get_next_hop_device_id(pkt->gen_device_id);
        dst_id = 0xFFFF; // Implement later
        if (dst_id == 0 || dst_id == 0xFFFF || dst_id == radio_get_device_id()) {
            LOG_ERR("No route to gen_device_id %d, cannot forward LARGE_DATA_RECEIVED", pkt->gen_device_id);
            return;
        }
        send_large_data_received(&recv_pkt, dst_id, DEVICE_TYPE_ANCHOR);
    }

    return;
}

void handle_large_data_chunk_ack(const large_data_chunk_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != radio_get_device_id()) {
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
    if (!ld_sender.active || ld_sender.dst_id != dst_id || ld_sender.gen_device_id != pkt->gen_device_id || ld_sender.data_id != pkt->data_id) {
        LOG_WRN("LARGE_DATA_CHUNK_ACK from %d but sender inactive or dst/gen/data mismatch", dst_id);
        return;
    }

    // Check transfer is complete or not
    if (pkt->hdr.status == STATUS_SUCCESS && pkt->page_index == (ld_sender.page_count - 1) && pkt->chunk_index == ((ld_sender.last_page_size + SEND_DATA_MAX - 1) / SEND_DATA_MAX - 1)) {
        LOG_INF("Large data transfer complete for gen %d data_id %d (status 0x%02x last_page_size %d page_count %d/%d chunk_index %d)", pkt->gen_device_id, pkt->data_id, pkt->hdr.status, ld_sender.last_page_size, ld_sender.page_count, pkt->page_index, pkt->chunk_index);
        ld_sender.active = false;
        free_large_data_slot(find_large_data_slot(ld_sender.gen_device_id, ld_sender.data_id));
        return;
    }

    switch (DEVICE_TYPE) {
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
                    ld_sender.next_chunk = 0;
                    large_data_init_t init_pkt = {
                        .gen_device_id = pkt->gen_device_id,
                        .data_id = pkt->data_id,
                        .total_size = ld_sender.total_size,
                        .page_count = ld_sender.page_count,
                        .last_page_size = ld_sender.last_page_size,
                        .crc32 = ld_sender.crc32,
                    };
                    send_large_data_init(&init_pkt, dst_id, pkt->hdr.device_type, PACKET_PRIORITY_HIGH);
                } else if (pkt->hdr.status == STATUS_FAILURE) {
                    // Rebuild same chunk and resend
                    LOG_WRN("Received LARGE_DATA_CHUNK_ACK with status 0x%02x, resending chunk %d for gen %d data_id %d", pkt->hdr.status, pkt->chunk_index, pkt->gen_device_id, pkt->data_id);
                    uint8_t idx = pkt->chunk_index;
                    uint32_t addr = ld_sender.base_addr + (uint32_t)(pkt->page_index * 20 + idx) * SEND_DATA_MAX;
                    uint8_t csz = chunk_size_for_large_data(pkt->page_index * 20 + idx);
                    int err = psram_read(addr, chunk_pkt.data, csz);
                    if (err) {
                        LOG_ERR("psram_read @0x%06x failed (%d), cannot resend chunk", addr, err);
                        return;
                    }
                    chunk_pkt.gen_device_id = pkt->gen_device_id;
                    chunk_pkt.data_id = pkt->data_id;
                    chunk_pkt.page_index = pkt->page_index;
                    chunk_pkt.chunk_index = idx;
                    send_large_data_chunk(&chunk_pkt, dst_id, pkt->hdr.device_type, ld_sender.priority);
                } else if (pkt->hdr.status == STATUS_SUCCESS || pkt->hdr.status == STATUS_ALREADY_EXISTS) {
                    // Send next chunk if ack is success
                    send_next_large_data_chunk(dst_id, pkt->hdr.device_type);
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

void handle_large_data_received(const large_data_receive_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
    // Only Process if it's for this device
    if (pkt->hdr.device_id != radio_get_device_id()) {
        return;
    }

    // Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
    if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
        LOG_WRN("Unknown device type 0x%02x in LARGE_DATA_RECEIVED from %d, rejecting", pkt->hdr.device_type, dst_id);
        return;
    }

    LOG_INF("Received LARGE_DATA_RECEIVED from %s ID:%d for DATA ID:%d", device_type_str(pkt->hdr.device_type), dst_id, pkt->data_id);

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Gateway will never receive large data received because only anchor can receive data received, so just ignore if received
            return;
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
                large_data_receive_t recv_pkt = {
                    .gen_device_id = pkt->gen_device_id,
                    .data_id = pkt->data_id,
                };
                // Find dst_id from route table using gen_device_id and send large data received packet to sensor
                // dst_id = get_next_hop_device_id(pkt->gen_device_id);
                dst_id = 0xFFFF; // Implement later
                if (dst_id == 0 || dst_id == 0xFFFF || dst_id == radio_get_device_id()) {
                    LOG_ERR("No route to gen_device_id %d, cannot forward LARGE_DATA_RECEIVED", pkt->gen_device_id);
                    return;
                }
                send_large_data_received(&recv_pkt, dst_id, DEVICE_TYPE_SENSOR);
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
        slots[i].active = false;
    }

    for (int i = 0; i < LARGE_DATA_SLOT_COUNT; i++) {
        is_ld_slot_empty[i] = true;
    }

    LOG_INF("Large Data module initialized with %d slots at PSRAM 0x%06x-0x%06x", LARGE_DATA_RECEIVER_SLOT_COUNT, LARGE_DATA_PSRAM_BASE, LARGE_DATA_PSRAM_BASE + LARGE_DATA_PSRAM_SIZE - 1);

    return 0;
}

void large_data_tick(void)
{
    for (int i = 0; i < LARGE_DATA_RECEIVER_SLOT_COUNT; i++) {
        if (slots[i].active && nbtimeout_expired(&slots[i].idle_timeout)) {
            LOG_WRN("Large data slot for gen %d data_id %d timed out, freeing slot", slots[i].gen_device_id, slots[i].data_id);
            free_large_data_slot(i);
        } else if (slots[i].active && slots[i].upstream_ready && DEVICE_TYPE == DEVICE_TYPE_ANCHOR && !ld_sender.active) {
            // Need to Upstream
            LOG_INF("Starting to upstream large data for gen %d data_id %d to gateway", slots[i].gen_device_id, slots[i].data_id);
            infra_entry_t entry;
            int err = storage_infra_get(0, &entry);
            if (err) {
                LOG_ERR("Failed to get infra info from storage (%d), cannot upstream chunk", err);
                continue;
            }
            ld_sender.active = true;
            ld_sender.dst_id = entry.device_id;
            ld_sender.gen_device_id = slots[i].gen_device_id;
            ld_sender.data_id = slots[i].data_id;
            ld_sender.priority = slots[i].priority;
            ld_sender.total_size = slots[i].total_size;
            ld_sender.page_count = slots[i].page_count;
            ld_sender.last_page_size = slots[i].last_page_size;
            ld_sender.total_chunks = slots[i].total_chunks;
            ld_sender.crc32 = slots[i].crc32;
            ld_sender.base_addr = slots[i].base_addr;
            ld_sender.next_chunk = 0;
            large_data_init_t init_pkt = {
                .gen_device_id = slots[i].gen_device_id,
                .data_id = slots[i].data_id,
                .total_size = slots[i].total_size,
                .page_count = slots[i].page_count,
                .last_page_size = slots[i].last_page_size,
                .crc32 = slots[i].crc32,
            };
            send_large_data_init(&init_pkt, entry.device_id, entry.device_type, slots[i].priority);
        }
    }
}