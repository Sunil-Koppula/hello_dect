
#include <stdint.h>
#include <stddef.h>

/*
 * Sensor application-layer payload schema (shared with host tools).
 *
 * These structures are serialized over the wire (they ride inside the
 * REPORT_*, CONFIG_*, and LARGE_DATA_* transport packets defined in
 * src/protocol.h) and are also parsed by the Python host tools in tools/.
 * To keep the byte layout identical across the sensor firmware, the
 * gateway, and the host:
 *
 *   1. Every serialized struct is __attribute__((packed)) — no padding.
 *   2. Every field uses a fixed-width integer type. The enums below are
 *      kept only as named constant sets; struct fields never use a bare
 *      enum type, whose width is implementation-defined (int by default,
 *      2 bytes under ARM EABI -fshort-enums) and would otherwise differ
 *      between the firmware and an x86 host.
 *   3. _Static_assert guards lock each struct size so an accidental
 *      field/width change fails to compile instead of corrupting the wire
 *      format.
 *
 * NOTE: sensor_large_data_structure_t is ~200 KB. It must always be staged
 * in PSRAM (PSRAM_LARGE_DATA_BASE) — never instantiated on the stack or in
 * internal RAM.
 */

#define SENSOR_REPORT_INFO_MAX 16
#define SENSOR_CONFIG_INFO_MAX 16
#define SENSOR_LARGE_DATA_INFO_MAX 200 * 1024 // 200 KB max for large data info (e.g. sound data)
#define SENSOR_DATA_STR_SN_MAX 12
#define SENSOR_REPORT_CONFIG_NAME_MAX 6
#define SENSOR_REPORT_CONFIG_MASK 0xEFFF

typedef enum
{
    SENSOR_REPORT_TYPE_UNKNOWN = 0x0000,
    SENSOR_REPORT_TYPE_3100 = 0x3100,
    SENSOR_REPORT_TYPE_3101 = 0x3101,
    SENSOR_REPORT_TYPE_3102 = 0x3102,
    SENSOR_REPORT_TYPE_3103 = 0x3103,
    SENSOR_REPORT_TYPE_3104 = 0x3104,
    SENSOR_REPORT_TYPE_3105 = 0x3105,
    SENSOR_REPORT_TYPE_3200 = 0x3200, // Reserved for Smart Window Sensor
    SENSOR_REPORT_TYPE_3300 = 0x3300, // DECT NR+ sensor
    SENSOR_REPORT_TYPE_3500 = 0x3500, // H2S-5000
    SENSOR_REPORT_TYPE_3600 = 0x3600, // H2S-6000
    SENSOR_REPORT_TYPE_3800 = 0x3800, // HY-GUARD
    GATEWAY_SENSOR_INTERNAL_REPORT_TYPE_2100 = 0x2100
} sensor_report_type_t;

typedef enum
{
    SENSOR_LARGE_DATA_TYPE_UNKNOWN = 0x0000,
    SENSOR_LARGE_DATA_TYPE_3300 = 0x3300, // DECT NR+ sensor large data (e.g. sound data)
} sensor_large_data_type_t;

typedef enum
{
    SENSOR_ALARM_FLAG_NONE = 0x00,
    SENSOR_ALARM_FLAG_BATTERY = (1 << 0),
    SENSOR_ALARM_FLAG_TEMPERATURE1 = (1 << 1),
    SENSOR_ALARM_FLAG_TEMPERATURE2 = (1 << 2),
    SENSOR_ALARM_FLAG_HUMIDITY1 = (1 << 3),
    SENSOR_ALARM_FLAG_HUMIDITY2 = (1 << 4),
    SENSOR_ALARM_FLAG_GAS1 = (1 << 5),
    SENSOR_ALARM_FLAG_GAS2 = (1 << 6),
    SENSOR_ALARM_FLAG_GAS3 = (1 << 7),
    SENSOR_ALARM_FLAG_CURRENT = (1 << 8),
    SENSOR_ALARM_FLAG_ULTRASOUND = (1 << 9),
    SENSOR_ALARM_FLAG_VIBRATION = (1 << 10),
    SENSOR_ALARM_FLAG_MAX = (1 << 15)
} sensor_alarm_flags_t;

typedef enum
{
    SENSOR_REPORT_CONFIG_FLAG_DEFAULT = 0x00, // Encrypted
    SENSOR_REPORT_CONFIG_FLAG_NOT_ENCRYPTED = (1 << 0),
    SENSOR_REPORT_CONFIG_FLAG_DEMO_MODE = (1 << 1),
    SENSOR_REPORT_CONFIG_FLAG_VALID = (1 << 2)
} sensor_report_config_flags_t;

typedef enum
{
    SENSOR_CONFIG_CMD_NO = 0x00,
    SENSOR_CONFIG_CMD_LOGS = (1 << 0),
    SENSOR_CONFIG_CMD_LOGS_CLEAR = (1 << 1),
    SENSOR_CONFIG_CMD_DEMO_MODE = (1 << 2),
    SENSOR_CONFIG_CMD_RESET = (1 << 5),
    SENSOR_CONFIG_CMD_DELETE_CERTIFICATE = (1 << 6),
    SENSOR_CONFIG_CMD_OTA = (1 << 7)
} sensor_config_cmd_t;

typedef struct
{
    int16_t temperature1;
    uint16_t humidity1;
} __attribute__((packed)) sensor_report_info_3300_t;

typedef struct
{
    sensor_report_info_3300_t report_info;
    uint8_t  sound_record[SENSOR_LARGE_DATA_INFO_MAX];
} __attribute__((packed)) sensor_large_data_info_3300_t;

typedef struct
{
    int8_t temperature_max1;
    int8_t temperature_min1;
    uint8_t humidity_max1;
    uint8_t humidity_min1;
    uint8_t  random_number;
} __attribute__((packed)) sensor_config_info_3300_t;

typedef struct
{
    char     name[SENSOR_REPORT_CONFIG_NAME_MAX]; // ESC33/00
    char     sn[SENSOR_DATA_STR_SN_MAX];  // 123456789012 // HEX
    uint16_t report_type;             // sensor_report_type_t
    uint16_t firmware_version;
    uint8_t  battery_level;           // Battery percent 0~100
    uint16_t alarm_flags;             // sensor_alarm_flags_t bitmask (up to bit 15)
    uint8_t  report_flags;            // sensor_report_config_flags_t bitmask
    uint16_t report_crc16;            // covers the fixed header above
    union
    {   uint8_t  report_info[SENSOR_REPORT_INFO_MAX];    // Maximum bytes in reported data = 16
        sensor_report_info_3300_t report_info_3300;
    };

} __attribute__((packed)) sensor_data_structure_t;

typedef struct
{
    char     name[SENSOR_REPORT_CONFIG_NAME_MAX]; // ESC21/00   // Gateway name
    uint8_t  dest_id[6];  // Destination ID (SN of the Sensor)
    uint8_t  command;                 // sensor_config_cmd_t bitmask
    uint16_t new_firmware_version; // New Firmware Version For OTA
    uint8_t battery_level_min; // Battery minimum level for alarm
    uint16_t sleep_time_sec;  // Sleep time in seconds
    uint8_t  config_flags;            // sensor_report_config_flags_t bitmask
    uint16_t config_crc16;            // covers the fixed header above
    union
    {   uint8_t  config_info[SENSOR_CONFIG_INFO_MAX];
        sensor_config_info_3300_t config_info_3300;
    };

} __attribute__((packed)) sensor_config_structure_t;

typedef struct
{
    char     name[SENSOR_REPORT_CONFIG_NAME_MAX]; // ESC33/00
    char     sn[SENSOR_DATA_STR_SN_MAX];  // 123456789012 // HEX
    uint16_t data_type;               // sensor_large_data_type_t
    uint16_t firmware_version;
    uint8_t  battery_level;           // Battery percent 0~100
    uint32_t total_size;              // Total size of the large data info (can exceed 64 KB)
    uint16_t large_data_crc16;        // covers the fixed header above
    union
    {
        uint8_t  data_info[SENSOR_LARGE_DATA_INFO_MAX + SENSOR_REPORT_INFO_MAX];    // Maximum bytes in reported data = 16
        sensor_large_data_info_3300_t data_info_3300;
    };
} __attribute__((packed)) sensor_large_data_structure_t;

/*
 * Layout guards — lock the on-wire size of each structure. The fixed-header
 * byte count is spelled out so any field/width change is caught at compile
 * time instead of silently breaking interop with the host tools.
 */
_Static_assert(sizeof(sensor_data_structure_t) == 28 + SENSOR_REPORT_INFO_MAX,
               "sensor_data_structure_t layout changed");
_Static_assert(sizeof(sensor_config_structure_t) == 21 + SENSOR_CONFIG_INFO_MAX,
               "sensor_config_structure_t layout changed");
_Static_assert(sizeof(sensor_large_data_structure_t) ==
                   29 + (SENSOR_LARGE_DATA_INFO_MAX + SENSOR_REPORT_INFO_MAX),
               "sensor_large_data_structure_t layout changed");
