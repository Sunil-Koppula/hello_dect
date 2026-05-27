/*
 * Anchor pairing and main loop for DECT NR+ mesh network.
 *
 * The anchor is a hybrid device:
 *   - Upstream: pairs with gateway/other anchors (like a sensor, 4-step)
 *   - Downstream: accepts sensor pair requests (like a gateway)
 *
 * Tracking IDs are looked up from the tracker pool — no static TID variables.
 * Partition 1 stores upstream infra devices (max 8).
 * Partition 2 stores connected sensors.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include "main_sub.h"
#include "protocol.h"
#include "product_info.h"
#include "mesh.h"
#include "radio.h"
#include "storage.h"
#include "queue.h"
#include "tracker.h"
#include "data.h"
#include "large_data.h"
#include "config.h"
#include "log_color.h"
#include "slm_at_main.h"

LOG_MODULE_REGISTER(anchor, CONFIG_ANCHOR_LOG_LEVEL);

static int anchor_init(void)
{
	device_info_update();
	LOG_INF_BLU("%s Initialized with ID: %d SN: 0x%016llx, Hop: %d Mesh Devices: %d", device_type_str(DEVICE_TYPE), radio_get_device_id(), SERIAL_NUMBER, DEVICE_HOP_NUMBER, MESH_DEVICES_COUNT);

	tracker_init();
	data_init();
	config_init();
	large_data_init();
	
	if (infra_count > 0) {
		if (infra_count >= MAX_ANCHORS) {
			return 0;
		}
	}

	send_pair_request();
	return 0;
}

static void anchor_process_rx(const uint8_t *data, uint16_t sender_id, int16_t rssi_2)
{
	switch (data[0]) {
	case PACKET_PAIR_REQUEST:
		handle_pair_request((const pair_request_t *)data, sender_id, rssi_2);
		break;
	
	case PACKET_PAIR_RESPONSE:
		handle_pair_response((const pair_response_t *)data, sender_id, rssi_2);
		break;

	case PACKET_PAIR_CONFIRM:
		handle_pair_confirm((const pair_confirm_t *)data, sender_id, rssi_2);
		break;
	
	case PACKET_PAIR_ACK:
		handle_pair_ack((const pair_ack_t *)data, sender_id, rssi_2);
		break;

	case PACKET_JOINED_NETWORK:
		handle_joined_network((const joined_network_t *)data, sender_id, rssi_2);
		break;
	
	case PACKET_JOINED_NETWORK_ACK:
		handle_joined_network_ack((const joined_network_ack_t *)data, sender_id, rssi_2);
		break;
	
	case PACKET_PING_DEVICE:
		handle_ping_device((const ping_device_t *)data, sender_id, rssi_2);
		break;

	case PACKET_PING_ACK:
		handle_ping_ack((const ping_ack_t *)data, sender_id, rssi_2);
		break;
	
	case PACKET_DEVICE_UPDATED:
		handle_device_updated((const device_updated_t *)data, sender_id, rssi_2);
		break;

	case PACKET_DEVICE_UPDATED_ACK:
		handle_device_updated_ack((const device_updated_ack_t *)data, sender_id, rssi_2);
		break;

	case PACKET_REPAIR_REQUEST:
		handle_repair_request((const repair_request_t *)data, sender_id, rssi_2);
		break;

	case PACKET_REPAIR_RESPONSE:
		handle_repair_response((const repair_response_t *)data, sender_id, rssi_2);
		break;

	case PACKET_ROUTE_DISCOVERY:
		handle_route_discovery((const route_discovery_t *)data, sender_id, rssi_2);
		break;

	case PACKET_ROUTE_DISCOVERY_ACK:
		handle_route_discovery_ack((const route_discovery_ack_t *)data, sender_id, rssi_2);
		break;

	case PACKET_ROUTE_INFO:
		handle_route_info((const route_info_t *)data, sender_id, rssi_2);
		break;

	case PACKET_ROUTE_INFO_ACK:
		handle_route_info_ack((const route_info_ack_t *)data, sender_id, rssi_2);
		break;

	case PACKET_DATA_INIT:
		handle_data_init((const data_init_t *)data, sender_id, rssi_2);
		break;

	case PACKET_DATA_INIT_ACK:
		handle_data_init_ack((const data_init_ack_t *)data, sender_id, rssi_2);
		break;

	case PACKET_DATA_CHUNK:
		handle_data_chunk((const data_chunk_t *)data, sender_id, rssi_2);
		break;

	case PACKET_DATA_CHUNK_ACK:
		handle_data_chunk_ack((const data_chunk_ack_t *)data, sender_id, rssi_2);
		break;

	case PACKET_DATA_RECEIVED:
		handle_data_received((const data_receive_t *)data, sender_id, rssi_2);
		break;
		
	case PACKET_CONFIG:
		handle_config((const config_t *)data, sender_id, rssi_2);
		break;

	case PACKET_CONFIG_ACK:
		handle_config_ack((const config_ack_t *)data, sender_id, rssi_2);
		break;

	case PACKET_CONFIG_RECEIVED:
		handle_config_received((const config_received_t *)data, sender_id, rssi_2);
		break;

	case PACKET_LARGE_DATA_INIT:
		handle_large_data_init((const large_data_init_t *)data, sender_id, rssi_2);
		break;

	case PACKET_LARGE_DATA_INIT_ACK:
		handle_large_data_init_ack((const large_data_init_ack_t *)data, sender_id, rssi_2);
		break;

	case PACKET_LARGE_DATA_CHUNK:
		handle_large_data_chunk((const large_data_chunk_t *)data, sender_id, rssi_2);
		break;

	case PACKET_LARGE_DATA_CHUNK_ACK:
		handle_large_data_chunk_ack((const large_data_chunk_ack_t *)data, sender_id, rssi_2);
		break;

	case PACKET_LARGE_DATA_RECEIVED:
		handle_large_data_received((const large_data_receive_t *)data, sender_id, rssi_2);
		break;

	default:
		break;
	}

	return;
}

void anchor_main(void)
{
	int err;
	main_sub_state_t state = MAIN_SUB_INIT;

	while (1) {
		switch (state) {
			case MAIN_SUB_INIT:
			{
				if (SERIAL_NUMBER == 0xFFFFFFFFFFFFFFFF || SERIAL_NUMBER == 0) {
					// LOG_WRN("Serial Number not set, waiting");
					state = MAIN_SUB_SLM_AT;
					break;
				}
				err = anchor_init();
				if (err) {
					LOG_ERR("Anchor init failed, err %d", err);
					state = MAIN_SUB_ERROR;
					break;
				}
				state = MAIN_SUB_RX_WINDOW;
			}
			break;

			case MAIN_SUB_RX_WINDOW:
			{
				err = receive(1, 25);
				if (err) {
					LOG_ERR("Reception failed, err %d", err);
					state = MAIN_SUB_ERROR;
					break;
				}
				k_sem_take(&operation_sem, K_FOREVER);
				state = MAIN_SUB_RX_PROCESS;
			}
			break;

			case MAIN_SUB_RX_PROCESS:
			{
				struct rx_data_item rx_item;
				int rx_count = 0;

				while (rx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
					rx_queue_get(&rx_item, K_NO_WAIT) == 0) {
					anchor_process_rx(rx_item.data, rx_item.sender_id, rx_item.rssi_2);
					rx_count++;
				}
				state = MAIN_SUB_TX_PROCESS;
			}
			break;

			case MAIN_SUB_TX_PROCESS:
			{
				struct tx_data_item tx_large_item;
				int tx_count = 0;

				while (tx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
					tx_queue_get(&tx_large_item, K_NO_WAIT) == 0) {
					err = transmit(0, tx_large_item.data, tx_large_item.data_len, 0);
					if (err) {
						LOG_ERR("TX failed, err %d", err);
						state = MAIN_SUB_ERROR;
						break;
					}
					k_sem_take(&operation_sem, K_FOREVER);
					tx_count++;
				}
				state = MAIN_SUB_SLM_AT;
			}
			break;

			case MAIN_SUB_SLM_AT:
			{
				slm_at_run_cycle();
				if (SERIAL_NUMBER == 0xFFFFFFFFFFFFFFFF || SERIAL_NUMBER == 0) {
					state = MAIN_SUB_INIT;
					break;
				}
				state = MAIN_SUB_TRACKER;
			}
			break;

			case MAIN_SUB_TRACKER:
			{
				mesh_tick();
				tracker_tick(tracker_default_expired_cb);
				state = MAIN_SUB_CONFIG;
			}
			break;

			case MAIN_SUB_CONFIG:
			{
				config_tick();
				state = MAIN_SUB_REPORT;
			}
			break;

			case MAIN_SUB_REPORT:
			{
				data_tick();
				state = MAIN_SUB_LARGE_DATA;
			}
			break;

			case MAIN_SUB_LARGE_DATA:
			{
				large_data_tick();
				state = MAIN_SUB_OTA;
			}
			break;

			case MAIN_SUB_OTA:
			{
				// Implement later
				state = MAIN_SUB_PING_DEVICES;
			}
			break;

			case MAIN_SUB_PING_DEVICES:
			{
				state = MAIN_SUB_RX_WINDOW;
			}
			break;

			case MAIN_SUB_ERROR:
			{
				LOG_ERR("Anchor in error state, waiting 10s before retry");
				k_msleep(10000);
				state = MAIN_SUB_INIT;
			}
			break;

			default:
			{
				LOG_ERR("Anchor in unknown state %d", state);
				// Reset the device to recover from unknown state
				sys_reboot(SYS_REBOOT_COLD);
			}
			break;
		}
	}
}
