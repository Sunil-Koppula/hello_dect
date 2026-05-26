/*
 * Gateway main loop and pairing handler for DECT NR+ mesh network.
 *
 * Handles the gateway/anchor side of the 4-step pairing:
 *   Sensor → Gateway:  PAIR_REQUEST   → handle_pair_request() sends PAIR_RESPONSE
 *   Sensor → Gateway:  PAIR_CONFIRM   → gateway sends PAIR_ACK, stores sensor
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

LOG_MODULE_REGISTER(gateway, CONFIG_GATEWAY_LOG_LEVEL);

/* Broadcast SYNC_TIME every N tracker ticks (~once every 10s at the current cadence). */
#define SYNC_TIME_BROADCAST_PERIOD_TICKS 200

static int gateway_init(void)
{
	device_info_update();
	LOG_INF_BLU("%s Initialized with ID: %d SN: 0x%016llx, Hop: %d Mesh Devices: %d", device_type_str(DEVICE_TYPE), radio_get_device_id(), SERIAL_NUMBER, DEVICE_HOP_NUMBER, MESH_DEVICES_COUNT);

	tracker_init();
	mesh_time_init();
	data_init();
	config_init();
	large_data_init();

	if (storage_mesh_count() > 0) {
		LOG_INF("------------DEVICES IN MESH STORAGE------------");

		for (int i = 0; i < storage_mesh_count(); i++) {
			mesh_entry_t entry;
			int err = storage_mesh_get(i, &entry);
			if (err) {
				LOG_ERR("Failed to read mesh entry %d, err %d", i, err);
				continue;
			}
			if (entry.device_type == DEVICE_TYPE_SENSOR) {
				LOG_INF("%d: %s ID:%d SN:%lld V:%d CID:%d", i,
					device_type_str(entry.device_type), entry.device_id, entry.serial_num, entry.version,
					entry.connected_device_id);
			} else {
				LOG_INF("%d: %s ID:%d SN:%lld V:%d hop:%d SEN_CNT:%d", i,
					device_type_str(entry.device_type), entry.device_id, entry.serial_num, entry.version,
					entry.hop_num, entry.sensor_count);
			}
		}
	}

	ping_known_devices(0, STATUS_SUCCESS);

	return 0;
}

static void gateway_process_rx(const uint8_t *data, uint16_t sender_id, int16_t rssi_2)
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
}

void gateway_main(void)
{
	int err;
	main_sub_state_t state = MAIN_SUB_INIT;

	while (1) {
		switch (state) {
		case MAIN_SUB_INIT:
		{
			err = gateway_init();
			if (err) {
				LOG_ERR("Gateway init failed, err %d", err);
				state = MAIN_SUB_ERROR;
				break;
			}
			state = MAIN_SUB_RX_WINDOW;
			break;
		}

		case MAIN_SUB_RX_WINDOW:
		{
			err = receive(1, 20);
			if (err) {
				LOG_ERR("Reception failed, err %d", err);
				state = MAIN_SUB_ERROR;
				break;
			}
			k_sem_take(&operation_sem, K_FOREVER);
			state = MAIN_SUB_RX_PROCESS;
			break;
		}

		case MAIN_SUB_RX_PROCESS:
		{
			struct rx_data_item rx_item;
			int rx_count = 0;

			while (rx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
			    rx_queue_get(&rx_item, K_NO_WAIT) == 0) {
				gateway_process_rx(rx_item.data, rx_item.sender_id, rx_item.rssi_2);
				rx_count++;
			}
			
			state = MAIN_SUB_TX_PROCESS;
			break;
		}

		case MAIN_SUB_TX_PROCESS:
		{
			struct tx_data_item tx_item;
			int tx_count = 0;

			while (tx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
			       tx_queue_get(&tx_item, K_NO_WAIT) == 0) {
				err = transmit(0, tx_item.data, tx_item.data_len, 0);
				if (err) {
					LOG_ERR("TX failed, err %d", err);
					break;
				}
				k_sem_take(&operation_sem, K_FOREVER);
				tx_count++;
			}
			
			state = MAIN_SUB_TRACKER;
			break;
		}

		case MAIN_SUB_TRACKER:
			tracker_tick(tracker_default_expired_cb);
			data_tick();
			large_data_tick();
			known_devices_tick();
			// mesh_time_check_milestone();
			slm_at_run_cycle();
			state = MAIN_SUB_RX_WINDOW;
			break;

		case MAIN_SUB_ERROR:
			LOG_ERR("Gateway in error state, waiting 10s before retry");
			k_msleep(10000);
			state = MAIN_SUB_INIT;
			break;

		default:
			LOG_ERR("Gateway in unknown state %d", state);
			// Reset the device to recover from unknown state
			sys_reboot(SYS_REBOOT_COLD);
			break;
		}
	}
}
