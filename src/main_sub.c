/* Main Sub */

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

LOG_MODULE_REGISTER(main_sub, CONFIG_MAIN_SUB_LOG_LEVEL);

static bool is_main_sub_initialized = false;

static int main_sub_init(void)
{
    if (is_main_sub_initialized) {
        return 0;
    }

    // Update device info
    device_info_update();
	LOG_INF_BLU("%s Initialized with ID: %d SN: 0x%016llx, Hop: %d", device_type_str(DEVICE_TYPE), radio_get_device_id(), SERIAL_NUMBER, DEVICE_HOP_NUMBER);

    // Initialize subsystems
    tracker_init();
    mesh_time_init();
    data_init();
    config_init();
    large_data_init();

    switch (DEVICE_TYPE) {
        case DEVICE_TYPE_GATEWAY:
        {
            // Ping known devices
            ping_known_devices(0, STATUS_SUCCESS);
        }
        break;

        case DEVICE_TYPE_ANCHOR:
        {
            if (infra_count > 0) {
                for (int i = 0; i < infra_count; i++) {
                    LOG_INF("Already paired with %s ID: %d", device_type_str(infra_devices[i].entry.device_type), infra_devices[i].entry.device_id);
                }
                if (infra_count >= MAX_ANCHORS) {
                    return 0;
                }
            }
            send_pair_request();
        }
        break;

        case DEVICE_TYPE_SENSOR:
        {
            if (infra_count > 0) {
                LOG_INF("Already paired with %s ID: %d", device_type_str(infra_devices[0].entry.device_type), infra_devices[0].entry.device_id);
                return 0;
            }
            /* Not paired — start pairing. */
            LOG_INF("Sensor not paired!");
            send_pair_request();
        }
        break;

        default:
        {
            LOG_ERR("Invalid device type %d", DEVICE_TYPE);
            return -EINVAL;
        }
        break;
    }

    is_main_sub_initialized = true;
    return 0;
}

static void process_rx(const uint8_t *data, uint16_t sender_id, int16_t rssi_2)
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

		case PACKET_REPORT_INIT:
			handle_report_init((const report_init_t *)data, sender_id, rssi_2);
			break;

		case PACKET_REPORT_INIT_ACK:
			handle_report_init_ack((const report_init_ack_t *)data, sender_id, rssi_2);
			break;

		case PACKET_REPORT_CHUNK:
			handle_report_chunk((const report_chunk_t *)data, sender_id, rssi_2);
			break;

		case PACKET_REPORT_CHUNK_ACK:
			handle_report_chunk_ack((const report_chunk_ack_t *)data, sender_id, rssi_2);
			break;

		case PACKET_REPORT_RECEIVED:
			handle_report_received((const report_received_t *)data, sender_id, rssi_2);
			break;

        case PACKET_REPORT_RECEIVED_ACK:
            // handle_report_received_ack((const report_received_ack_t *)data, sender_id, rssi_2);
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

        case PACKET_CONFIG_RECEIVED_ACK:
            // handle_config_received_ack((const config_received_ack_t *)data, sender_id, rssi_2);
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
			handle_large_data_received((const large_data_received_t *)data, sender_id, rssi_2);
			break;

        case PACKET_LARGE_DATA_RECEIVED_ACK:
            // handle_large_data_received_ack((const large_data_received_ack_t *)data, sender_id, rssi_2);
            break;

        case PACKET_OTA_INIT:
            // handle_ota_init((const ota_init_t *)data, sender_id, rssi_2);
            break;

        case PACKET_OTA_INIT_ACK:
            // handle_ota_init_ack((const ota_init_ack_t *)data, sender_id, rssi_2);
            break;
        
        case PACKET_OTA_CHUNK:
            // handle_ota_chunk((const ota_chunk_t *)data, sender_id, rssi_2);
            break;

        case PACKET_OTA_CHUNK_ACK:
            // handle_ota_chunk_ack((const ota_chunk_ack_t *)data, sender_id, rssi_2);
            break;

        case PACKET_OTA_RECEIVED:
            // handle_ota_received((const ota_received_t *)data, sender_id, rssi_2);
            break;

         case PACKET_OTA_RECEIVED_ACK:
            // handle_ota_received_ack((const ota_received_ack_t *)data, sender_id, rssi_2);
            break;

		default:
			break;
	}
}

void main_sub_run(void)
{
    int rc = 0;
    static main_sub_state_t main_sub_state = MAIN_SUB_INIT;
    static main_sub_state_t main_sub_state_debug = MAIN_SUB_INIT;

    if (main_sub_state != main_sub_state_debug) {
        LOG_DBG("Main Sub State: %d", main_sub_state);
        main_sub_state_debug = main_sub_state;
    }

    switch (main_sub_state) {
        case MAIN_SUB_INIT:
        {
            if (SERIAL_NUMBER == 0xFFFFFFFFFFFFFFFF || SERIAL_NUMBER == 0) {
                main_sub_state = MAIN_SUB_INIT;
                break;
            }
            rc = main_sub_init();
            if (rc) {
                LOG_ERR("Main Sub init failed, err %d", rc);
                main_sub_state = MAIN_SUB_ERROR;
                break;
            }
            main_sub_state = MAIN_SUB_RX_WINDOW;
        }
        break;

        case MAIN_SUB_RX_WINDOW:
        {
            rc = receive(1, 25);
            if (rc) {
                LOG_ERR("Reception failed, err %d", rc);
                main_sub_state = MAIN_SUB_ERROR;
                break;
            }
            k_sem_take(&operation_sem, K_FOREVER);
            main_sub_state = MAIN_SUB_RX_PROCESS;
        }
        break;

        case MAIN_SUB_RX_PROCESS:
        {
            struct rx_data_item rx_item;
            int rx_count = 0;

            while (rx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
                rx_queue_get(&rx_item, K_NO_WAIT) == 0) {
                process_rx(rx_item.data, rx_item.sender_id, rx_item.rssi_2);
                rx_count++;
            }
            main_sub_state = MAIN_SUB_TX_PROCESS;
        }
        break;

        case MAIN_SUB_TX_PROCESS:
        {
            struct tx_data_item tx_item;
            int tx_count = 0;

            while (tx_count < MAX_QUEUE_PROCESS_PER_CYCLE &&
                tx_queue_get(&tx_item, K_NO_WAIT) == 0) {
                rc = transmit(0, tx_item.data, tx_item.data_len, 0);
                if (rc) {
                    LOG_ERR("TX failed, err %d", rc);
                    main_sub_state = MAIN_SUB_ERROR;
                    break;
                }
                k_sem_take(&operation_sem, K_FOREVER);
                tx_count++;
            }
            main_sub_state = MAIN_SUB_RX_WINDOW;
        }
        break;

        case MAIN_SUB_ERROR:
        {
            LOG_ERR("Main Sub in error state, waiting 10s before retry");
            k_msleep(10000);
            is_main_sub_initialized = false;
            main_sub_state = MAIN_SUB_INIT;
        }
        break;

        default:
        {
            LOG_ERR("Main Sub in unknown state %d", main_sub_state);
            // Reset the device to recover from unknown state
            sys_reboot(SYS_REBOOT_COLD);
        }
        break;
    }
}