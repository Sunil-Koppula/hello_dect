/*
 * Wire protocol definitions for DECT NR+ mesh network
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* Device Type */
typedef enum {
	DEVICE_TYPE_UNKNOWN = 0x00,
	DEVICE_TYPE_GATEWAY = 0x01,
	DEVICE_TYPE_ANCHOR  = 0x02,
	DEVICE_TYPE_SENSOR  = 0x03,
} device_type_t;

/* Packet Types Identifier */
typedef enum {
	PACKET_UNKNOWN       		= 0x00,
	PACKET_PAIR_REQUEST  		= 0x01,
	PACKET_PAIR_RESPONSE 		= 0x02,
	PACKET_PAIR_CONFIRM  		= 0x03,
	PACKET_PAIR_ACK       		= 0x04,
	PACKET_JOINED_NETWORK       = 0x05,
	PACKET_JOINED_NETWORK_ACK  	= 0x06,
	PACKET_PING_DEVICE			= 0x07,
	PACKET_PING_ACK				= 0x08,
} packet_type_t;

/* Packet Priority Levels */
#define PACKET_PRIORITY_HIGH    0x00
#define PACKET_PRIORITY_LOW     0x01

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
#define STATUS_STORAGE_FULL             0x0D
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

/********** Packet Structures **********/

/* Pairing Request Packet */
typedef struct {
	packet_header_t hdr;
	uint32_t random_num;
} __attribute__((packed)) pair_request_t;

#define PAIR_REQUEST_PACKET_SIZE sizeof(pair_request_t)

/* Pairing Response Packet — unicast to requester */
typedef struct {
	packet_header_t hdr;
	uint32_t hash;
	uint8_t hop_num;
} __attribute__((packed)) pair_response_t;

/* Pairing Confirm Packet — unicast to responder */
typedef struct {
	packet_header_t hdr;
	uint16_t version;
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
	uint8_t hop_num;
} __attribute__((packed)) joined_network_t;

#define JOINED_NETWORK_PACKET_SIZE sizeof(joined_network_t)

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