/*
 * Wire protocol definitions for DECT NR+ mesh network
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define MAX_DEPTH           8
#define MAX_ANCHORS         8
#define MAX_SENSORS         64
#define MAX_DEVICES         256
#define MAX_KNOWN_DEVICES   (MAX_ANCHORS + MAX_SENSORS)

/* Device Type */
typedef enum {
	DEVICE_TYPE_UNKNOWN = 0x00,
	DEVICE_TYPE_GATEWAY = 0x01,
	DEVICE_TYPE_ANCHOR  = 0x02,
	DEVICE_TYPE_SENSOR  = 0x03,
} device_type_t;

/* Data Types */
typedef enum {
	DATA_TYPE_UNKNOWN	= 0x00,
	DATA_TYPE_REPORT	= 0x01,
	DATA_TYPE_CONFIG	= 0x02,
	DATA_TYPE_LARGE		= 0x03,
	DATA_TYPE_OTA		= 0x04,
} data_type_t;

/* Packet Types Identifier */
typedef enum {
	PACKET_UNKNOWN       			= 0x00,
	PACKET_PAIR_REQUEST  			= 0x01,
	PACKET_PAIR_RESPONSE 			= 0x02,
	PACKET_PAIR_CONFIRM  			= 0x03,
	PACKET_PAIR_ACK       			= 0x04,
	PACKET_JOINED_NETWORK       	= 0x05,
	PACKET_JOINED_NETWORK_ACK  		= 0x06,
	PACKET_PING_DEVICE				= 0x07,
	PACKET_PING_ACK					= 0x08,
	PACKET_DEVICE_UPDATED			= 0x09,
	PACKET_DEVICE_UPDATED_ACK		= 0x0A,
	PACKET_REPAIR_REQUEST			= 0x0B,
	PACKET_REPAIR_RESPONSE			= 0x0C,
	PACKET_ROUTE_DISCOVERY			= 0x0D,
	PACKET_ROUTE_DISCOVERY_ACK		= 0x0E,
	PACKET_ROUTE_INFO				= 0x0F,
	PACKET_ROUTE_INFO_ACK			= 0x10,
	PACKET_REPORT_INIT				= 0x11,
	PACKET_REPORT_INIT_ACK			= 0x12,
	PACKET_REPORT_CHUNK				= 0x13,
	PACKET_REPORT_CHUNK_ACK			= 0x14,
	PACKET_REPORT_RECEIVED			= 0x15,
	PACKET_REPORT_RECEIVED_ACK		= 0x16,
	PACKET_CONFIG					= 0x17,
	PACKET_CONFIG_ACK				= 0x18,
	PACKET_CONFIG_RECEIVED			= 0x19,
	PACKET_CONFIG_RECEIVED_ACK		= 0x1A,
	PACKET_LARGE_DATA_INIT			= 0x1B,
	PACKET_LARGE_DATA_INIT_ACK		= 0x1C,
	PACKET_LARGE_DATA_CHUNK			= 0x1D,
	PACKET_LARGE_DATA_CHUNK_ACK		= 0x1E,
	PACKET_LARGE_DATA_RECEIVED		= 0x1F,
	PACKET_LARGE_DATA_RECEIVED_ACK	= 0x20,
	PACKET_OTA_INIT					= 0x21,
	PACKET_OTA_INIT_ACK				= 0x22,
	PACKET_OTA_CHUNK				= 0x23,
	PACKET_OTA_CHUNK_ACK			= 0x24,
	PACKET_OTA_RECEIVED				= 0x25,
	PACKET_OTA_RECEIVED_ACK			= 0x26,
} packet_type_t;

/* Packet Priority Levels */
#define PACKET_PRIORITY_HIGH    0x00
#define PACKET_PRIORITY_MEDIUM  0x01
#define PACKET_PRIORITY_LOW     0x02

/* General Status Codes — unified across all packet types */
#define STATUS_SUCCESS                  0x00
#define STATUS_FAILURE                  0x01
#define STATUS_CRC_FAIL                 0x02
#define STATUS_TIMEOUT                  0x03
#define STATUS_RESOURCE_UNAVAILABLE     0x04
#define STATUS_INVALID_PARAMETER        0x05
#define STATUS_NOT_SUPPORTED            0x06
#define STATUS_REJECTED                 0x07
#define STATUS_ALREADY_EXISTS           0x08
#define STATUS_NOT_FOUND                0x09
#define STATUS_BUSY                     0x0A
#define STATUS_VERSION_MISMATCH         0x0B
#define STATUS_DEVICE_JOINED            0x0C
#define STATUS_STORAGE_FULL             0x0D
#define STATUS_DEVICE_REMOVED		   	0x0E
#define STATUS_AUTH_FAILED              0x0F
#define STATUS_COMPLETE                 0x10
#define STATUS_VENDOR_SPECIFIC          0x1F

/********** Common Packet Header **********/

/* Common header — first 7 bytes of every packet. */
typedef struct {
	uint8_t packet_type;        /* packet_type_t */
	uint8_t device_type;        /* device_type_t */
	uint8_t priority;           /* PACKET_PRIORITY_HIGH / MEDIUM / LOW */
	uint8_t tracking_id;        /* Tracking ID (1–254) */
	uint16_t device_id;         /* sender or target device ID */
	uint8_t status;             /* STATUS_SUCCESS / STATUS_FAILURE */
} __attribute__((packed)) packet_header_t;

#define PACKET_HEADER_SIZE sizeof(packet_header_t)

/* Route Info Entry */
typedef struct {
	uint16_t device_id;		/* short device ID of the device */
	uint8_t hop_num;		/* hop count from gateway */
} __attribute__((packed)) route_info_entry_t;

/********** Packet Structures **********/

/* Pairing Request Packet */
typedef struct {
	packet_header_t hdr;
	uint32_t random_num;
	uint32_t hash;
} __attribute__((packed)) pair_request_t;

#define PAIR_REQUEST_PACKET_SIZE sizeof(pair_request_t)

/* Pairing Response Packet — unicast to requester */
typedef struct {
	packet_header_t hdr;
	uint8_t hop_num;
} __attribute__((packed)) pair_response_t;

/* Pairing Confirm Packet — unicast to responder */
typedef struct {
	packet_header_t hdr;
	uint16_t version;
	uint8_t hop_num;
} __attribute__((packed)) pair_confirm_t;

#define PAIR_CONFIRM_PACKET_SIZE sizeof(pair_confirm_t)

/* Pairing Acknowledgment Packet — unicast to confirmer */
typedef struct {
	packet_header_t hdr;
	uint16_t version;
	uint8_t hop_num;
} __attribute__((packed)) pair_ack_t;

#define PAIR_ACK_PACKET_SIZE sizeof(pair_ack_t)

/* JOINED NETWORK Packet */
typedef struct {
	packet_header_t hdr;
	uint8_t device_type;			/* device_type_t */
	uint16_t device_id;				/* short device ID */
	uint64_t serial_num;			/* 64-bit serial number */
	uint16_t version;				/* device firmware version */
	uint16_t connected_device_id;	/* parent/connected device ID */
	uint8_t hop_num; 				/* hop count from gateway */
	uint8_t sensor_count; 			/* number of sensors connected to this device (for gateway/anchor) */
} __attribute__((packed)) joined_network_t;

#define JOINED_NETWORK_PACKET_SIZE sizeof(joined_network_t)

/* JOINED NETWORK ACK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t dst_device_id;			/* short device ID of the device that sent the JOINED_NETWORK packet being acknowledged */
	uint8_t dst_device_type;			/* device_type_t of the device that sent the JOINED_NETWORK packet being acknowledged */
} __attribute__((packed)) joined_network_ack_t;

#define JOINED_NETWORK_ACK_PACKET_SIZE sizeof(joined_network_ack_t)

/* PING DEVICE Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t dst_device_id;			/* short device ID of the device being pinged */
	uint8_t hop_num; 				/* hop count from gateway */
	uint16_t version;				/* device firmware version */
	uint16_t total_devices;			/* total number of devices in the network (for gateway) or behind this anchor (for anchor) */
	uint64_t timestamp;				/* sender timestamp for RTT measurement */
} __attribute__((packed)) ping_device_t;

#define PING_DEVICE_PACKET_SIZE sizeof(ping_device_t)

/* PING ACK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t dst_device_id;			/* short device ID of the device being pinged */
	uint8_t hop_num; 				/* hop count from gateway */
	uint16_t version;				/* device firmware version */
	uint64_t timestamp;				/* sender timestamp for RTT measurement */
} __attribute__((packed)) ping_ack_t;

#define PING_ACK_PACKET_SIZE sizeof(ping_ack_t)

/* DEVICE UPDATED Packet */
typedef struct {
	packet_header_t hdr;
	uint8_t device_type;			/* device_type_t */
	uint16_t device_id;				/* short device ID of the device that was updated */
	uint64_t serial_num;			/* 64-bit serial number */
	uint16_t version;				/* device firmware version */
	uint16_t connected_device_id;	/* parent/connected device ID */
	uint8_t hop_num; 				/* hop count from gateway */
	uint8_t sensor_count; 			/* number of sensors connected to this device (for gateway/anchor) */
} __attribute__((packed)) device_updated_t;

#define DEVICE_UPDATED_PACKET_SIZE sizeof(device_updated_t)

/* DEVICE UPDATED ACK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t dst_device_id;			/* short device ID of the device that sent the DEVICE_UPDATED packet being acknowledged */
	uint8_t dst_device_type;			/* device_type_t of the device that sent the DEVICE_UPDATED packet being acknowledged */
} __attribute__((packed)) device_updated_ack_t;

#define DEVICE_UPDATED_ACK_PACKET_SIZE sizeof(device_updated_ack_t)

/* REPAIR REQUEST Packet */
typedef struct {
	packet_header_t hdr;
	uint32_t random_num;
	uint32_t hash;
	uint16_t version;
	uint8_t hop_num;
} __attribute__((packed)) repair_request_t;

#define REPAIR_REQUEST_PACKET_SIZE sizeof(repair_request_t)

/* REPAIR RESPONSE Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t version;
	uint8_t hop_num;
} __attribute__((packed)) repair_response_t;

#define REPAIR_RESPONSE_PACKET_SIZE sizeof(repair_response_t)

/* ROUTE DISCOVERY Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t device_id;
	uint8_t device_type;
	uint8_t hop_num;
	uint8_t data_type;
	uint16_t data_id;
} __attribute__((packed)) route_discovery_t;

#define ROUTE_DISCOVERY_PACKET_SIZE sizeof(route_discovery_t)

/* ROUTE DISCOVERY ACK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t device_id;
	uint8_t device_type;
	uint8_t hop_num;
} __attribute__((packed)) route_discovery_ack_t;

#define ROUTE_DISCOVERY_ACK_PACKET_SIZE sizeof(route_discovery_ack_t)

/* ROUTE INFO Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t device_id;
	uint8_t device_type;
	uint8_t data_type;
	uint16_t data_id;
	route_info_entry_t route_info[MAX_DEPTH];
} __attribute__((packed)) route_info_t;

#define ROUTE_INFO_PACKET_SIZE sizeof(route_info_t)

/* ROUTE INFO ACK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t device_id;				/* short device ID of the device whose route info is being acknowledged */
	uint8_t device_type;			/* device_type_t of the device whose route info is being acknowledged */
	uint8_t data_type;				/* data_type_t of the data being acknowledged */
	uint16_t data_id;				/* ID of the data being acknowledged */
} __attribute__((packed)) route_info_ack_t;

#define ROUTE_INFO_ACK_PACKET_SIZE sizeof(route_info_ack_t)

#define SEND_DATA_MAX 180 /* Max data size per chunk: keeps data_chunk_t (190 B with 10 B header overhead) within 14 DECT subslots @ MCS 2 with margin. */

/* REPORT INIT Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t gen_device_id;			/* short device ID of the device that generated this data (e.g. a sensor) */
	uint8_t data_id;				/* ID of the data being sent (for the sender's reference, e.g. to match with ACKs) */
	uint16_t total_size;			/* total size of the data being sent (can be larger than what fits in one packet) */
	uint8_t chunk_count;			/* total number of chunks that will be sent */
	uint8_t last_chunk_size;		/* size of the last chunk (since it may be smaller than the others) */
	uint32_t crc32;					/* CRC32 of the entire data for integrity checking */
} __attribute__((packed)) report_init_t;

#define REPORT_INIT_PACKET_SIZE sizeof(report_init_t)

/* REPORT INIT ACK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t gen_device_id;			/* short device ID of the device that generated this data (e.g. a sensor) */
	uint8_t data_id;				/* ID of the data being acknowledged */
} __attribute__((packed)) report_init_ack_t;

#define REPORT_INIT_ACK_PACKET_SIZE sizeof(report_init_ack_t)

/* REPORT CHUNK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t gen_device_id;			/* short device ID of the device that generated this data (e.g. a sensor) */
	uint8_t data_id;				/* ID of the data being sent (for the sender's reference, e.g. to match with ACKs) */
	uint8_t chunk_index;			/* index of this chunk (starting from 0) */
	uint8_t data[SEND_DATA_MAX];	/* chunk data */
} __attribute__((packed)) report_chunk_t;

#define REPORT_CHUNK_PACKET_SIZE sizeof(report_chunk_t)

/* REPORT CHUNK ACK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t gen_device_id;			/* short device ID of the device that generated this data (e.g. a sensor) */
	uint8_t data_id;				/* ID of the data being acknowledged */
	uint8_t chunk_index;			/* index of the chunk being acknowledged */
} __attribute__((packed)) report_chunk_ack_t;

#define REPORT_CHUNK_ACK_PACKET_SIZE sizeof(report_chunk_ack_t)

/* REPORT RECEIVED Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t gen_device_id;			/* short device ID of the device that generated this data (e.g. a sensor) */
	uint8_t data_id;				/* ID of the data being received */
	route_info_entry_t route_info[MAX_DEPTH];
} __attribute__((packed)) report_received_t;

#define REPORT_RECEIVED_PACKET_SIZE sizeof(report_received_t)

/* REPORT RECEIVED ACK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t gen_device_id;			/* short device ID of the device that generated this data (e.g. a sensor) */
	uint8_t data_id;				/* ID of the data being acknowledged */
} __attribute__((packed)) report_received_ack_t;

#define REPORT_RECEIVED_ACK_PACKET_SIZE sizeof(report_received_ack_t)

/* LARGE DATA INIT Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t gen_device_id;			/* short device ID of the device that generated this data (e.g. a sensor) */
	uint8_t data_id;				/* ID of the data being sent (for the sender's reference, e.g. to match with ACKs) */
	uint32_t total_size;			/* total size of the data being sent (can be larger than what fits in one packet) */
	uint8_t page_count;				/* total number of pages that will be sent */
	uint16_t last_page_size;		/* size of the last page (since it may be smaller than the others) */
	uint32_t crc32;					/* CRC32 of the entire data for integrity checking */
} __attribute__((packed)) large_data_init_t;

/* LARGE DATA INIT ACK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t gen_device_id;			/* short device ID of the device that generated this data (e.g. a sensor) */
	uint8_t data_id;				/* ID of the data being acknowledged */
} __attribute__((packed)) large_data_init_ack_t;

#define LARGE_DATA_INIT_ACK_PACKET_SIZE sizeof(large_data_init_ack_t)

/* LARGE DATA CHUNK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t gen_device_id;			/* short device ID of the device that generated this data (e.g. a sensor) */
	uint8_t data_id;				/* ID of the data being sent (for the sender's reference, e.g. to match with ACKs) */
	uint8_t page_index;				/* index of this page (starting from 0) */
	uint8_t chunk_index;			/* index of this chunk within the page (starting from 0) */
	uint8_t data[SEND_DATA_MAX];	/* chunk data */
} __attribute__((packed)) large_data_chunk_t;

#define LARGE_DATA_CHUNK_PACKET_SIZE sizeof(large_data_chunk_t)

/* LARGE DATA CHUNK ACK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t gen_device_id;			/* short device ID of the device that generated this data (e.g. a sensor) */
	uint8_t data_id;				/* ID of the data being acknowledged */
	uint8_t page_index;				/* index of the page being acknowledged */
	uint8_t chunk_index;			/* index of the chunk within the page being acknowledged */
} __attribute__((packed)) large_data_chunk_ack_t;

#define LARGE_DATA_CHUNK_ACK_PACKET_SIZE sizeof(large_data_chunk_ack_t)

/* LARGE DATA RECEIVED Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t gen_device_id;			/* short device ID of the device that generated this data (e.g. a sensor) */
	uint8_t data_id;				/* ID of the data being received */
} __attribute__((packed)) large_data_received_t;

#define LARGE_DATA_RECEIVED_PACKET_SIZE sizeof(large_data_received_t)

/* LARGE DATA RECEIVED ACK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t gen_device_id;			/* short device ID of the device that generated this data (e.g. a sensor) */
	uint8_t data_id;				/* ID of the data being acknowledged */
} __attribute__((packed)) large_data_received_ack_t;

#define LARGE_DATA_RECEIVED_ACK_PACKET_SIZE sizeof(large_data_received_ack_t)

#define MAX_CONFIG_SIZE 128

/* CONFIG Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t dst_device_id;						/* short device ID of the device being configured */
	uint8_t dst_device_type;					/* device_type_t of the device being configured */
	uint8_t data_type;							/* data_type_t of the config data */
	uint16_t data_id;							/* ID of the config data (for the sender's reference, e.g. to match with ACKs) */
	route_info_entry_t route_info[MAX_DEPTH];	/* routing info for the config packet to reach the destination device (filled by sender based on route discovery or routing table) */
	uint8_t config_len; 						/* length of the config data */
	uint32_t config_crc32;						/* CRC32 of the config data for integrity checking */
	uint8_t config[MAX_CONFIG_SIZE];			/* config data */
} __attribute__((packed)) config_t;

#define CONFIG_PACKET_SIZE sizeof(config_t)

/* CONFIG ACK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t dst_device_id;				/* short device ID of the device being configured */
	uint8_t dst_device_type;			/* device_type_t of the device being configured */
} __attribute__((packed)) config_ack_t;

#define CONFIG_ACK_PACKET_SIZE sizeof(config_ack_t)

/* CONFIG RECEIVED Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t dst_device_id;			/* short device ID of the device being configured */
	uint8_t dst_device_type;		/* device_type_t of the device being configured */
	uint8_t data_type;				/* data_type_t of the config data */
	uint16_t data_id;				/* ID of the config data */
} __attribute__((packed)) config_received_t;

#define CONFIG_RECEIVED_PACKET_SIZE sizeof(config_received_t)

/* CONFIG RECEIVED ACK Packet */
typedef struct {
	packet_header_t hdr;
	uint16_t dst_device_id;			/* short device ID of the device being configured */
	uint8_t dst_device_type;		/* device_type_t of the device being configured */
	uint8_t data_type;				/* data_type_t of the config data */
	uint16_t data_id;				/* ID of the config data */
} __attribute__((packed)) config_received_ack_t;

#define CONFIG_RECEIVED_ACK_PACKET_SIZE sizeof(config_received_ack_t)

/* Get device type as string */
static inline const char *device_type_str(device_type_t type)
{
	switch (type) {
	case DEVICE_TYPE_GATEWAY: return "GATEWAY";
	case DEVICE_TYPE_ANCHOR:  return "ANCHOR";
	case DEVICE_TYPE_SENSOR:  return "SENSOR";
	default:                  return "UNKNOWN";
	}
}

#endif /* PROTOCOL_H */