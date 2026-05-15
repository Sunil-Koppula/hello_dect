#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include "config.h"
#include "mesh.h"
#include "queue.h"
#include "radio.h"
#include "tracker.h"
#include "timeout.h"
#include "psram.h"
#include "product_info.h"
#include "storage.h"
#include "main_sub.h"

LOG_MODULE_REGISTER(config, CONFIG_CONFIG_LOG_LEVEL);

struct config_slot config_slots[CONFIG_SLOT_COUNT];

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------**** Config Slots ****------------------------------------------------------------------------------ */

static uint32_t config_slot_psram_addr(int idx)
{
	return CONFIG_PSRAM_BASE + ((uint32_t)idx * CONFIG_MAX_SIZE);
}

static int find_config_slot(uint16_t dst_device_id, int *idx_out)
{
	for (int i = 0; i < CONFIG_SLOT_COUNT; i++) {
		if (config_slots[i].active && config_slots[i].dst_device_id == dst_device_id) {
			*idx_out = i;
			return i;
		}
	}
	*idx_out = -1;
	return -1;
}

static int alloc_config_slot(void)
{
	for (int i = 0; i < CONFIG_SLOT_COUNT; i++) {
		if (!config_slots[i].active) {
			return i;
		}
	}
	return -1;
}

static void config_slot_free(int idx)
{
	config_slots[idx].active = false;
}

static uint8_t validate_config(const config_t *pkt)
{
	int idx;
	int ret = find_config_slot(pkt->dst_device_id, &idx);
	if (ret < 0) {
		idx = alloc_config_slot();
		if (idx < 0) {
			LOG_WRN("CONFIG rejected: no free config slot");
			return STATUS_RESOURCE_UNAVAILABLE;
		}

		config_slots[idx].active = true;
		config_slots[idx].is_sent = false;
		config_slots[idx].dst_device_id = pkt->dst_device_id;
		config_slots[idx].dst_device_type = pkt->dst_device_type;
		config_slots[idx].config_len = pkt->config_len;
		config_slots[idx].config_crc32 = pkt->config_crc32;

		// Verify CRC of received bytes first (no PSRAM round-trip needed).
		uint32_t calc_crc32 = crc32_ieee(pkt->config, pkt->config_len);
		if (calc_crc32 != pkt->config_crc32) {
			LOG_WRN("CONFIG rejected: CRC32 mismatch (expected 0x%08X, got 0x%08X)", pkt->config_crc32, calc_crc32);
			config_slot_free(idx);
			return STATUS_CRC_FAIL;
		}

		// CRC OK — persist to PSRAM.
		uint32_t addr = config_slot_psram_addr(idx);
		int err = psram_write(addr, pkt->config, pkt->config_len);
		if (err) {
			LOG_ERR("psram_write @0x%06x failed (%d), aborting config store", addr, err);
			config_slot_free(idx);
			return STATUS_FAILURE;
		}
	} else {
		LOG_WRN("CONFIG rejected: config slot already active for dst_device_id 0x%04X", pkt->dst_device_id);
		return STATUS_ALREADY_EXISTS;
	}

	return STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------------**** TX Helpers ****------------------------------------------------------------------------------- */

int send_config(config_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority)
{
	pkt->hdr.packet_type = PACKET_CONFIG;
	pkt->hdr.device_type = DEVICE_TYPE;
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracker_next_id();
	pkt->hdr.device_id = dst_id;

    // Add tracker entry for retries
	tracker_add(dst_id, radio_get_device_id(), pkt->hdr.tracking_id, PACKET_CONFIG, PACKET_TIMEOUT_MS, PACKET_MAX_RETRIES, pkt, sizeof(*pkt));

	LOG_INF("----> Sending CONFIG to device %s ID:%d", device_type_str(dst_type), dst_id);
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_config_ack(config_ack_t *pkt, uint16_t dst_id, uint8_t dst_type, uint8_t priority, uint8_t tracking_id)
{
	pkt->hdr.packet_type = PACKET_CONFIG_ACK;
	pkt->hdr.device_type = DEVICE_TYPE;
	pkt->hdr.priority = priority;
	pkt->hdr.tracking_id = tracking_id;
	pkt->hdr.device_id = dst_id;

	LOG_INF("----> Sending CONFIG_ACK to device %s ID:%d", device_type_str(dst_type), dst_id);
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

int send_config_received(config_received_t *pkt, uint16_t dst_id, uint8_t dst_type)
{
	pkt->hdr.packet_type = PACKET_CONFIG_RECEIVED;
	pkt->hdr.device_type = DEVICE_TYPE;
	pkt->hdr.priority = PACKET_PRIORITY_HIGH;
	pkt->hdr.tracking_id = tracker_next_id();
	pkt->hdr.device_id = dst_id;

	LOG_INF("----> Sending CONFIG_RECEIVED to device %s ID:%d", device_type_str(dst_type), dst_id);
	return tx_queue_put(pkt, sizeof(*pkt), pkt->hdr.priority);
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------**** Handlers Functions ****--------------------------------------------------------------------------- */

void handle_config(const config_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
	// Only Process if it's for this device
	if (pkt->hdr.device_id != radio_get_device_id()) {
		return;
	}

	// Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
	if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
		LOG_WRN("Unknown device type 0x%02x in CONFIG from %d, rejecting", pkt->hdr.device_type, dst_id);
		return;
	}

	LOG_INF("Received CONFIG from %s ID:%d config_len %d crc 0x%08x", device_type_str(pkt->hdr.device_type), dst_id, pkt->config_len, pkt->config_crc32);

	config_ack_t ack = {
		.dst_device_id = pkt->dst_device_id,
		.dst_device_type = pkt->dst_device_type,
	};

	if (pkt->config_len == 0 || pkt->config_len > CONFIG_MAX_SIZE) {
		LOG_WRN("CONFIG rejected: invalid config_len %d", pkt->config_len);
		ack.hdr.status = STATUS_INVALID_PARAMETER;
		send_config_ack(&ack, dst_id, pkt->hdr.device_type, pkt->hdr.priority, pkt->hdr.tracking_id);
		return;
	}

	switch (DEVICE_TYPE) {
		case DEVICE_TYPE_GATEWAY:
		{
			// Gateway will not process any config, so reject any incoming config packet
			return;
		}
		break;

		case DEVICE_TYPE_ANCHOR:
		case DEVICE_TYPE_SENSOR:
		{
			if (pkt->hdr.device_type == DEVICE_TYPE_GATEWAY || pkt->hdr.device_type == DEVICE_TYPE_ANCHOR) {
				// validate config packet and store it if valid, if not reject the packet
				uint8_t status = validate_config(pkt);
				ack.hdr.status = status;
			} else {
				// Reject config packet except from gateway and anchor
				return;
			}
		}
		break;

		default:
		{
			// There are only 3 valid device types, reject any config if this device has invalid type
			return;
		}
		break;
	}

	send_config_ack(&ack, dst_id, pkt->hdr.device_type, pkt->hdr.priority, pkt->hdr.tracking_id);

	if (ack.hdr.status == STATUS_SUCCESS && DEVICE_TYPE == DEVICE_TYPE_SENSOR) {
		// Send config received packet to confirm the config is received and processed
		config_received_t recv_pkt = {
			.dst_device_id = pkt->dst_device_id,
			.dst_device_type = pkt->dst_device_type,
		};
		send_config_received(&recv_pkt, dst_id, pkt->hdr.device_type);
	}

	return;
}

void handle_config_ack(const config_ack_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
	// Only Process if it's for this device
	if (pkt->hdr.device_id != radio_get_device_id()) {
		return;
	}

	// Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
	if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
		LOG_WRN("Unknown device type 0x%02x in CONFIG_ACK from %d, rejecting", pkt->hdr.device_type, dst_id);
		return;
	}

	LOG_INF("Received CONFIG_ACK from %s ID:%d status 0x%02x", device_type_str(pkt->hdr.device_type), dst_id, pkt->hdr.status);

	// Remove tracker
	tracker_remove_by_tracking_id(pkt->hdr.tracking_id);

	switch (DEVICE_TYPE) {
		case DEVICE_TYPE_GATEWAY:
		{
			if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
				int idx;
				int ret = find_config_slot(pkt->dst_device_id, &idx);
				if (pkt->hdr.status == STATUS_SUCCESS || pkt->hdr.status == STATUS_ALREADY_EXISTS) {
					// Mark slot as sent; slot is freed later when CONFIG_RECEIVED arrives.
					if (ret >= 0) {
						LOG_INF("CONFIG_ACK successful for device %s ID:%d (slot %d), waiting for CONFIG_RECEIVED", device_type_str(pkt->dst_device_type), pkt->dst_device_id, idx);
						config_slots[idx].is_sent = true;
					} else {
						LOG_WRN("CONFIG_ACK successful but no matching config slot found for device %s ID:%d", device_type_str(pkt->dst_device_type), pkt->dst_device_id);
					}
				} else {
					// resend config: read bytes back from PSRAM and re-send.
					if (ret >= 0) {
						LOG_WRN("CONFIG_ACK failed with status 0x%02x, resending config for device %s ID:%d", pkt->hdr.status, device_type_str(pkt->dst_device_type), pkt->dst_device_id);
						config_t config_pkt = {
							.dst_device_id = pkt->dst_device_id,
							.dst_device_type = pkt->dst_device_type,
							.config_len = config_slots[idx].config_len,
							.config_crc32 = config_slots[idx].config_crc32,
						};
						uint32_t addr = CONFIG_PSRAM_BASE + ((uint32_t)idx * CONFIG_MAX_SIZE);
						int err = psram_read(addr, config_pkt.config, config_slots[idx].config_len);
						if (err) {
							LOG_ERR("psram_read @0x%06x failed (%d), cannot resend config", addr, err);
							return;
						}
						send_config(&config_pkt, pkt->dst_device_id, pkt->dst_device_type, PACKET_PRIORITY_HIGH);
					} else {
						LOG_WRN("CONFIG_ACK failed with status 0x%02x and no matching config slot found for device %s ID:%d, cannot resend", pkt->hdr.status, device_type_str(pkt->dst_device_type), pkt->dst_device_id);
					}
				}
			} else {
				// Reject config ack except from anchor and sensor
				return;
			}
		}
		break;

		case DEVICE_TYPE_ANCHOR:
		{
			if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
				int idx;
				int ret = find_config_slot(pkt->dst_device_id, &idx);
				if (pkt->hdr.status == STATUS_SUCCESS || pkt->hdr.status == STATUS_ALREADY_EXISTS) {
					// free slot
					if (ret >= 0) {
						LOG_INF("CONFIG_ACK successful, freeing config slot %d for device %s ID:%d", idx, device_type_str(pkt->dst_device_type), pkt->dst_device_id);
						config_slot_free(idx);
					} else {
						LOG_WRN("CONFIG_ACK successful but no matching config slot found for device %s ID:%d", device_type_str(pkt->dst_device_type), pkt->dst_device_id);
					}
				} else {
					config_slots[idx].is_sent = false; // Mark slot as not sent to trigger resend on next CONFIG_ACK
				}
			} else {
				// Reject config ack except from anchor and sensor
				return;
			}
		}
		break;

		case DEVICE_TYPE_SENSOR:
		{
			// Sensor will not receive config ack because only gateway and anchor can receive config ack, so just ignore if received
			return;
		}
		break;

		default:
		{
			// There are only 3 valid device types, reject any config ack if this device has invalid type
			return;
		}
		break;
	}

	return;
}

void handle_config_received(const config_received_t *pkt, uint16_t dst_id, int16_t rssi_2)
{
	// Only Process if it's for this device
	if (pkt->hdr.device_id != radio_get_device_id()) {
		return;
	}

	// Validate responder type: only accept if it's from a valid device type (gateway, anchor, sensor)
	if (pkt->hdr.device_type != DEVICE_TYPE_GATEWAY && pkt->hdr.device_type != DEVICE_TYPE_ANCHOR && pkt->hdr.device_type != DEVICE_TYPE_SENSOR) {
		LOG_WRN("Unknown device type 0x%02x in CONFIG_RECEIVED from %d, rejecting", pkt->hdr.device_type, dst_id);
		return;
	}

	LOG_INF("Received CONFIG_RECEIVED from %s ID:%d for %s ID:%d", device_type_str(pkt->hdr.device_type), dst_id, device_type_str(pkt->dst_device_type), pkt->dst_device_id);

	switch (DEVICE_TYPE) {
		case DEVICE_TYPE_GATEWAY:
		{
			if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
				// Just log the config received packet, no need to process or store it in gateway
				LOG_INF("CONFIG_RECEIVED indicates device %s ID:%d received the config successfully", device_type_str(pkt->dst_device_type), pkt->dst_device_id);
				// Free config slot
				int idx;
				int ret = find_config_slot(pkt->dst_device_id, &idx);
				if (ret >= 0) {
					LOG_INF("Freeing config slot %d for device %s ID:%d based on CONFIG_RECEIVED", idx, device_type_str(pkt->dst_device_type), pkt->dst_device_id);
					config_slot_free(idx);
				} else {
					LOG_WRN("CONFIG_RECEIVED but no matching config slot found for device %s ID:%d", device_type_str(pkt->dst_device_type), pkt->dst_device_id);
				}
			} else {
				// Reject config received packet except from anchor and sensor
				return;
			}
		}
		break;

		case DEVICE_TYPE_ANCHOR:
		{
			if (pkt->hdr.device_type == DEVICE_TYPE_ANCHOR || pkt->hdr.device_type == DEVICE_TYPE_SENSOR) {
				LOG_INF("Forwarding CONFIG_RECEIVED to Gateway for device %s ID:%d", device_type_str(pkt->dst_device_type), pkt->dst_device_id);
				// Forward config received packet to gateway
				infra_entry_t entry;
				int err = storage_infra_get(0, &entry);
				if (err) {
					LOG_ERR("Failed to get infra entry from storage (%d), cannot forward CONFIG_RECEIVED", err);
					return;
				}
				config_received_t fwd_pkt = {
					.dst_device_id = pkt->dst_device_id,
					.dst_device_type = pkt->dst_device_type,
				};
				send_config_received(&fwd_pkt, entry.device_id, entry.device_type);
			} else {
				// Reject config received packet except from anchor and sensor
				return;
			}
		}
		break;

		case DEVICE_TYPE_SENSOR:
		{
			// Sensor will not receive config received packet because only gateway and anchor can receive config received packet, so just ignore if received
			return;
		}
		break;

		default:
		{
			// There are only 3 valid device types, reject any config received if this device has invalid type
			return;
		}
		break;
	}

	return;
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------**** Module Init / Tick ****--------------------------------------------------------------------------- */

int config_init(void)
{
    for (int i = 0; i < CONFIG_SLOT_COUNT; i++) {
        config_slots[i].active = false;
        config_slots[i].is_sent = false;
        config_slots[i].dst_device_id = 0;
        config_slots[i].dst_device_type = 0;
        config_slots[i].config_len = 0;
        config_slots[i].config_crc32 = 0;
    }

    LOG_INF("Config Module Initialized with %d slots (slot size=%d) at PSRAM 0x%06x-0x%06x",
        CONFIG_SLOT_COUNT, CONFIG_MAX_SIZE,
        CONFIG_PSRAM_BASE, CONFIG_PSRAM_BASE + CONFIG_PSRAM_SIZE - 1);

    return 0;
}

void config_tick(void)
{
    for (int i = 0; i < CONFIG_SLOT_COUNT; i++) {
        if (config_slots[i].active && !config_slots[i].is_sent && DEVICE_TYPE == DEVICE_TYPE_ANCHOR) {
            // Downstream the packet to sensor
            uint32_t addr = config_slot_psram_addr(i);
            config_t config_pkt = {
                .dst_device_id = config_slots[i].dst_device_id,
                .dst_device_type = config_slots[i].dst_device_type,
                .config_len = config_slots[i].config_len,
                .config_crc32 = config_slots[i].config_crc32,
            };
            int err = psram_read(addr, config_pkt.config, config_slots[i].config_len);
            if (err) {
                LOG_ERR("psram_read @0x%06x failed (%d), cannot send config", addr, err);
                continue;
            }
            uint16_t dst_id = get_next_hop_device_id(config_slots[i].dst_device_id);
            send_config(&config_pkt, dst_id, config_slots[i].dst_device_type, PACKET_PRIORITY_HIGH);
            config_slots[i].is_sent = true;
        }
    }
}