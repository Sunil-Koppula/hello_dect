/*
 * PHY radio operations and callbacks for DECT NR+ mesh network
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>
#include <zephyr/drivers/hwinfo.h>
#include "radio.h"
#include "queue.h"
#include "protocol.h"
#include "storage.h"

LOG_MODULE_REGISTER(radio, CONFIG_RADIO_LOG_LEVEL);

bool radio_exit;
static uint16_t device_id;
static uint64_t modem_time;

/* Header type 1, due to endianness the order is different than in the specification. */
struct phy_ctrl_field_common {
	uint32_t packet_length : 4;
	uint32_t packet_length_type : 1;
	uint32_t header_format : 3;
	uint32_t short_network_id : 8;
	uint32_t transmitter_id_hi : 8;
	uint32_t transmitter_id_lo : 8;
	uint32_t df_mcs : 3;
	uint32_t reserved : 1;
	uint32_t transmit_power : 4;
	uint32_t pad : 24;
};

/* Semaphore to synchronize modem calls. */
K_SEM_DEFINE(operation_sem, 0, 1);

K_SEM_DEFINE(deinit_sem, 0, 1);

/* Dect PHY config parameters. */
struct nrf_modem_dect_phy_config_params dect_phy_config_params = {
	.band_group_index = ((CONFIG_CARRIER >= 525 && CONFIG_CARRIER <= 551)) ? 1 : 0,
	.harq_rx_process_count = 4,
	.harq_rx_expiry_time_us = 5000000,
};

/* Callback after init operation. */
static void on_init(const struct nrf_modem_dect_phy_init_event *evt)
{
	if (evt->err) {
		LOG_ERR("Init failed, err %d", evt->err);
		radio_exit = true;
		return;
	}

	k_sem_give(&operation_sem);
}

/* Callback after deinit operation. */
static void on_deinit(const struct nrf_modem_dect_phy_deinit_event *evt)
{
	if (evt->err) {
		LOG_ERR("Deinit failed, err %d", evt->err);
		return;
	}

	k_sem_give(&deinit_sem);
}

static void on_activate(const struct nrf_modem_dect_phy_activate_event *evt)
{
	if (evt->err) {
		LOG_ERR("Activate failed, err %d", evt->err);
		radio_exit = true;
		return;
	}

	k_sem_give(&operation_sem);
}

static void on_deactivate(const struct nrf_modem_dect_phy_deactivate_event *evt)
{

	if (evt->err) {
		LOG_ERR("Deactivate failed, err %d", evt->err);
		return;
	}

	k_sem_give(&deinit_sem);
}

static void on_configure(const struct nrf_modem_dect_phy_configure_event *evt)
{
	if (evt->err) {
		LOG_ERR("Configure failed, err %d", evt->err);
		return;
	}

	k_sem_give(&operation_sem);
}

/* Callback after link configuration operation. */
static void on_link_config(const struct nrf_modem_dect_phy_link_config_event *evt)
{
	LOG_DBG("link_config cb time %"PRIu64" status %d", modem_time, evt->err);
}

static void on_radio_config(const struct nrf_modem_dect_phy_radio_config_event *evt)
{
	if (evt->err) {
		LOG_ERR("Radio config failed, err %d", evt->err);
		return;
	}

	k_sem_give(&operation_sem);
}

/* Callback after capability get operation. */
static void on_capability_get(const struct nrf_modem_dect_phy_capability_get_event *evt)
{
	LOG_DBG("capability_get cb time %"PRIu64" status %d", modem_time, evt->err);
}

static void on_bands_get(const struct nrf_modem_dect_phy_band_get_event *evt)
{
	LOG_DBG("bands_get cb status %d", evt->err);
}

static void on_latency_info_get(const struct nrf_modem_dect_phy_latency_info_event *evt)
{
	LOG_DBG("latency_info_get cb status %d", evt->err);
}

/* Callback after time query operation. */
static void on_time_get(const struct nrf_modem_dect_phy_time_get_event *evt)
{
	LOG_DBG("time_get cb time %"PRIu64" status %d", modem_time, evt->err);
}

static void on_cancel(const struct nrf_modem_dect_phy_cancel_event *evt)
{
	LOG_DBG("on_cancel cb status %d", evt->err);
	k_sem_give(&operation_sem);
}

/* Operation complete notification. */
static void on_op_complete(const struct nrf_modem_dect_phy_op_complete_event *evt)
{
	LOG_DBG("op_complete cb time %"PRIu64" status %d", modem_time, evt->err);
	k_sem_give(&operation_sem);
}

/* Sender ID from the most recent PCC event (PCC always fires before PDC). */
static uint16_t last_sender_id;

/* Physical Control Channel reception notification. */
static void on_pcc(const struct nrf_modem_dect_phy_pcc_event *evt)
{
	last_sender_id = evt->hdr.hdr_type_1.transmitter_id_hi << 8 |
			 evt->hdr.hdr_type_1.transmitter_id_lo;
	LOG_DBG("PCC from device %d (status: %d, handle: %d)",
		last_sender_id, evt->header_status, evt->handle);
}

/* Physical Control Channel CRC error notification. */
static void on_pcc_crc_err(const struct nrf_modem_dect_phy_pcc_crc_failure_event *evt)
{
	LOG_WRN("pcc_crc_err cb time %"PRIu64"", modem_time);
}

/*
 * Known device cache — ISR-safe (RAM only, no EEPROM access).
 * Updated from main thread via radio_update_known_devices().
 */
#define MAX_KNOWN_DEVICES (MAX_SENSORS + MAX_ANCHORS)

static uint16_t known_devices[MAX_KNOWN_DEVICES];
static int known_device_count;

void radio_update_known_devices(void)
{
	int count = 0;

	for (int i = 0; i < storage_infra_count() && count < MAX_KNOWN_DEVICES; i++) {
		infra_entry_t entry;
		if (storage_infra_get(i, &entry) == 0) {
			known_devices[count++] = entry.device_id;
		}
	}

	for (int i = 0; i < storage_sensor_count() && count < MAX_KNOWN_DEVICES; i++) {
		sensor_entry_t entry;
		if (storage_sensor_get(i, &entry) == 0) {
			known_devices[count++] = entry.device_id;
		}
	}

	known_device_count = count;
}

/* Check if a packet should be enqueued (ISR-safe — no EEPROM access). */
static bool should_enqueue(uint8_t packet_type, uint16_t sender_id)
{
	/* Control packets (pairing + repair) — always accept.
	 * REPAIR_REQUEST/RESPONSE must pass here because the peer may not be
	 * in the known_devices cache yet during a repair. */
	if ((packet_type >= PACKET_PAIR_REQUEST && packet_type <= PACKET_PAIR_ACK) ||
	    packet_type == PACKET_REPAIR_REQUEST ||
	    packet_type == PACKET_REPAIR_RESPONSE) {
		return true;
	}

	/* Check cached known devices. */
	for (int i = 0; i < known_device_count; i++) {
		if (known_devices[i] == sender_id) {
			return true;
		}
	}

	LOG_DBG("Dropping packet type 0x%02x from unknown device %d", packet_type, sender_id);
	return false;
}

/* Physical Data Channel reception notification. */
static void on_pdc(const struct nrf_modem_dect_phy_pdc_event *evt)
{
	LOG_DBG("PDC received, len %d, rssi_2 %d", evt->len, evt->rssi_2);

	if (evt->len < 1) {
		return;
	}

	uint8_t packet_type = ((const uint8_t *)evt->data)[0];

	if (!should_enqueue(packet_type, last_sender_id)) {
		return;
	}

	if (packet_type == PACKET_DATA_CHUNK) {
		struct rx_large_data_item item = {
			.sender_id = last_sender_id,
			.rssi_2 = evt->rssi_2,
			.data_len = (evt->len < QUEUE_DATA_MAX) ? evt->len : QUEUE_DATA_MAX,
		};
		memcpy(item.data, evt->data, item.data_len);
		rx_large_queue_put(&item, item.data[2]); /* The 3rd byte of the header contains Priority. */
	} else {
		struct rx_small_data_item item = {
		.sender_id = last_sender_id,
		.rssi_2 = evt->rssi_2,
		.data_len = (evt->len < QUEUE_DATA_MAX) ? evt->len : QUEUE_DATA_MAX,
		};
		memcpy(item.data, evt->data, item.data_len);
		rx_small_queue_put(&item, item.data[2]); /* The 3rd byte of the header contains Priority. */
	}
}

/* Physical Data Channel CRC error notification. */
static void on_pdc_crc_err(const struct nrf_modem_dect_phy_pdc_crc_failure_event *evt)
{
	LOG_WRN("PDC CRC error (handle: %d)", evt->handle);
}

/* RSSI measurement result notification. */
static void on_rssi(const struct nrf_modem_dect_phy_rssi_event *evt)
{
	LOG_DBG("rssi cb time %"PRIu64" carrier %d", modem_time, evt->carrier);
}

static void on_stf_cover_seq_control(const struct nrf_modem_dect_phy_stf_control_event *evt)
{
	LOG_WRN("Unexpectedly in %s\n", (__func__));
}

static void on_test_rf_tx_cw_ctrl(const struct nrf_modem_dect_phy_test_rf_tx_cw_control_event *evt)
{
	LOG_WRN("Unexpectedly in %s\n", (__func__));
}

void dect_phy_event_handler(const struct nrf_modem_dect_phy_event *evt)
{
	modem_time = evt->time;

	switch (evt->id) {
	case NRF_MODEM_DECT_PHY_EVT_INIT:
		on_init(&evt->init);
		break;
	case NRF_MODEM_DECT_PHY_EVT_DEINIT:
		on_deinit(&evt->deinit);
		break;
	case NRF_MODEM_DECT_PHY_EVT_ACTIVATE:
		on_activate(&evt->activate);
		break;
	case NRF_MODEM_DECT_PHY_EVT_DEACTIVATE:
		on_deactivate(&evt->deactivate);
		break;
	case NRF_MODEM_DECT_PHY_EVT_CONFIGURE:
		on_configure(&evt->configure);
		break;
	case NRF_MODEM_DECT_PHY_EVT_RADIO_CONFIG:
		on_radio_config(&evt->radio_config);
		break;
	case NRF_MODEM_DECT_PHY_EVT_COMPLETED:
		on_op_complete(&evt->op_complete);
		break;
	case NRF_MODEM_DECT_PHY_EVT_CANCELED:
		on_cancel(&evt->cancel);
		break;
	case NRF_MODEM_DECT_PHY_EVT_RSSI:
		on_rssi(&evt->rssi);
		break;
	case NRF_MODEM_DECT_PHY_EVT_PCC:
		on_pcc(&evt->pcc);
		break;
	case NRF_MODEM_DECT_PHY_EVT_PCC_ERROR:
		on_pcc_crc_err(&evt->pcc_crc_err);
		break;
	case NRF_MODEM_DECT_PHY_EVT_PDC:
		on_pdc(&evt->pdc);
		break;
	case NRF_MODEM_DECT_PHY_EVT_PDC_ERROR:
		on_pdc_crc_err(&evt->pdc_crc_err);
		break;
	case NRF_MODEM_DECT_PHY_EVT_TIME:
		on_time_get(&evt->time_get);
		break;
	case NRF_MODEM_DECT_PHY_EVT_CAPABILITY:
		on_capability_get(&evt->capability_get);
		break;
	case NRF_MODEM_DECT_PHY_EVT_BANDS:
		on_bands_get(&evt->band_get);
		break;
	case NRF_MODEM_DECT_PHY_EVT_LATENCY:
		on_latency_info_get(&evt->latency_get);
		break;
	case NRF_MODEM_DECT_PHY_EVT_LINK_CONFIG:
		on_link_config(&evt->link_config);
		break;
	case NRF_MODEM_DECT_PHY_EVT_STF_CONFIG:
		on_stf_cover_seq_control(&evt->stf_cover_seq_control);
		break;
	case NRF_MODEM_DECT_PHY_EVT_TEST_RF_TX_CW_CONTROL_CONFIG:
		on_test_rf_tx_cw_ctrl(&evt->test_rf_tx_cw_control);
		break;
	}
}

void radio_set_device_id(uint16_t id)
{
	device_id = id;
}

uint16_t radio_get_device_id(void)
{
	return device_id;
}

/* Send operation. */
int transmit(uint32_t handle, void *data, size_t data_len, uint8_t packet_length)
{
	int err;

	if (packet_length == 0) {
		/* Auto-calculate subslots needed.
		 * ~14 bytes per subslot at MCS 2 (QPSK 3/4). */
		packet_length = (data_len + 13) / 14;
		if (packet_length == 0) {
			packet_length = 1;
		}
	}
	if (packet_length > 15) {
		packet_length = 15;
	}

	struct phy_ctrl_field_common header = {
		.header_format = 0x0,
		.packet_length_type = 0x0,
		.packet_length = packet_length,
		.short_network_id = (CONFIG_NETWORK_ID & 0xff),
		.transmitter_id_hi = (device_id >> 8),
		.transmitter_id_lo = (device_id & 0xff),
		.transmit_power = CONFIG_TX_POWER,
		.reserved = 0,
		.df_mcs = CONFIG_MCS,
	};

	struct nrf_modem_dect_phy_tx_params tx_op_params = {
		.start_time = 0,
		.handle = handle,
		.network_id = CONFIG_NETWORK_ID,
		.phy_type = 0,
		.lbt_rssi_threshold_max = 0,
		.carrier = CONFIG_CARRIER,
		.lbt_period = NRF_MODEM_DECT_LBT_PERIOD_MAX,
		.phy_header = (union nrf_modem_dect_phy_hdr *)&header,
		.data = data,
		.data_size = data_len,
	};

	err = nrf_modem_dect_phy_tx(&tx_op_params);
	if (err != 0) {
		return err;
	}

	return 0;
}

/* Receive operation. */
int receive(uint32_t handle, uint32_t duration_ms)
{
	int err;

	struct nrf_modem_dect_phy_rx_params rx_op_params = {
		.start_time = 0,
		.handle = handle,
		.network_id = CONFIG_NETWORK_ID,
		.mode = NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS,
		.rssi_interval = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_OFF,
		.link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED,
		.rssi_level = -60,
		.carrier = CONFIG_CARRIER,
		.duration = duration_ms * NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ,
		.filter.short_network_id = CONFIG_NETWORK_ID & 0xff,
		.filter.is_short_network_id_used = 1,
		/* listen for everything (broadcast mode used) */
		.filter.receiver_identity = 0,
	};

	err = nrf_modem_dect_phy_rx(&rx_op_params);
	if (err != 0) {
		return err;
	}

	return 0;
}