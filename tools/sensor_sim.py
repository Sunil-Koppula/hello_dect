#!/usr/bin/env python3
"""Sensor simulator for the hello_dect firmware (downstream-MCU role).

This script emulates an *actual sensor's* application MCU sitting behind a
hello_dect device on its downstream UART. It models real sensor behaviour
driven by several independent timers:

  - BATTERY drains 1% every 5 minutes (floor 0).
  - READINGS (temperature / humidity) are regenerated every 2 minutes. At each
    of these checks the alarm conditions are evaluated; if NO alarm, the new
    readings are just buffered for the next interval report.
  - REPORTS (AT#REPORT) are sent every `sleep_time_sec` (taken from the most
    recent AT#CONFIG; before any config arrives, --default-interval is used).
  - ALARMS: the 2-minute parameter check sends an AT#REPORT immediately
    (high priority) whenever an alarm condition holds — battery <=
    battery_level_min, or temp/humidity outside the configured 3300 bounds.

On startup it PROVISIONS the device SN: it reads the current SN with AT#SN?,
and if it differs from --sn it sets it with AT#SN="<sn>". If the set fails
(ERROR / no confirmation) the script aborts, so reports are never sent with a
mismatched SN. Use --skip-sn-check to bypass.

It also acts as a CONFIG sink: it receives AT#CONFIG pushes, CRC-verifies them,
replies OK/ERROR, decodes the sensor_config_structure_t, and applies the new
sleep_time_sec / battery_level_min / command immediately.

Wire format (matches src/slm_at_main.c cmd_report + src/data.c emission):
    AT#REPORT="<SN16>","<ID4>","<LEN4>","<PRIO2>","<CRC8>","<DATA hex>"<CR><LF>
    AT#CONFIG="<SN16>","<ID4>","<LEN4>","<CRC8>","<DATA hex>"<CR><LF>
All header fields are uppercase hex; CRC32 is zlib.crc32 (== Zephyr crc32_ieee)
over the raw payload bytes.

Payloads are packed C structs from tools/sensor_structure.c:
    report     -> sensor_data_structure_t       (name "ESC33", report_type 0x3300)
    config     -> sensor_config_structure_t     (name "ESC21")
    large data -> sensor_large_data_structure_t (name "ESC33", data_type 0x3300)
The AT#LD large-data blob is a serialized sensor_large_data_structure_t: a
29-byte fixed header followed by data_info_3300 (a 14-byte sensor_report_info_
3300_t plus the raw sound_record samples).

Examples:
    # Run as a live 3300 sensor: drains battery, generates readings, reports
    python sensor_sim.py --port COM32 --sn 00F91200DEADBEEF

    # Faster demo timing for testing (battery 1%/30s, gen 12s, report 20s)
    python sensor_sim.py --port COM32 --sn 00F91200DEADBEEF \\
        --battery-period 30 --gen-period 12 --default-interval 20

Requires: pyserial  (pip install pyserial)
"""

from __future__ import annotations

import argparse
import queue
import random
import struct
import sys
import threading
import time
import zlib
from typing import Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.stderr.write("error: pyserial not installed. Run: pip install pyserial\n")
    sys.exit(1)


DEFAULT_BAUD = 1000000          # uart1 current-speed in the DK overlay
DEFAULT_TIMEOUT_S = 2.0

MAX_REPORT_SIZE = 256           # src/data.h MAX_REPORT_SIZE
MAX_CONFIG_SIZE = 128           # src/config.h MAX_CONFIG_SIZE

# AT#LD large-data transfer (sensor -> board over serial).
# Payload is hex-encoded in the AT line, so the framed command is
#   len("\r\nAT#LD=") + 6 quoted fields + 5 commas + 2*payload + "\r\n".
# Capped at 450 B/chunk to match the firmware's AT_DATA_PAYLOAD_MAX
# (src/slm_at_main.c) -> ~955-char line, safely under the firmware's
# SLM_UART_AT_COMMAND_LEN (1024, minus 1 for the NUL = 1023 usable).
AT_LD_PAYLOAD_MAX = 450         # bytes of binary payload per AT#LD chunk
LD_CHUNKS_PER_PAGE = 32         # chunk index rolls to next page after 32, matching
                                # LARGE_DATA_CHUNKS_PER_SIZE in src/large_data.h
LARGE_DATA_MAX_TRANSFER = 200 * 1024  # src/large_data.h LARGE_DATA_MAX_TRANSFER_SIZE
# Flow control: the firmware AT RX buffer cannot absorb back-to-back lines (it
# disables RX and wedges), so AT#LD chunks are sent one at a time, waiting for
# the device's OK/ERROR before the next. This is the max wait before proceeding
# anyway (degraded pacing) if no response arrives.
LD_ACK_TIMEOUT_S = 2.0

# Real-world default timings (overridable via CLI for testing).
BATTERY_PERIOD_S = 5 * 60       # -1% battery every 5 minutes
GEN_PERIOD_S     = 2 * 60       # regenerate readings every 2 minutes
DEFAULT_INTERVAL_S = 600        # sleep_time_sec before a config sets it (10 min)
DEFAULT_BATTERY_MIN = 30        # battery_level_min alarm threshold before config

# Identity strings from the spec (fit in char[6], see sensor_structure.c).
REPORT_NAME = "ESC33"
CONFIG_NAME = "ESC21"
REPORT_TYPE_3300 = 0x3300

# Large-data structure identity (sensor_large_data_structure_t).
LARGE_DATA_NAME = "ESC33"       # name[6], same convention as the report
LARGE_DATA_TYPE_3300 = 0x3300   # sensor_large_data_type_t

# report_flags / config_flags: 0x01 = NOT_ENCRYPTED (see sensor_structure.c).
FLAG_NOT_ENCRYPTED = 0x01

# AT#REPORT priority: low priority (2) when no alarm, high (0) on an alarm.
PRIO_NORMAL = 2
PRIO_ALARM = 0

# data_id auto-increments 1..DATA_ID_MAX then wraps back to 1 (never 0).
DATA_ID_MAX = 0xEFFF


def next_data_id(data_id: int) -> int:
    """Advance data_id, wrapping DATA_ID_MAX -> 1 (stays in 1..DATA_ID_MAX)."""
    return data_id + 1 if data_id < DATA_ID_MAX else 1

# Human-readable names for sensor_alarm_flags_t bits (for the dashboard).
ALARM_NAMES = [
    (1 << 0, "BATTERY"), (1 << 1, "TEMP1"), (1 << 2, "TEMP2"),
    (1 << 3, "HUM1"), (1 << 4, "HUM2"), (1 << 5, "GAS1"), (1 << 6, "GAS2"),
    (1 << 7, "GAS3"), (1 << 8, "CURRENT"), (1 << 9, "ULTRASOUND"),
    (1 << 10, "VIBRATION"),
]


def alarm_text(flags: int) -> str:
    names = [n for bit, n in ALARM_NAMES if flags & bit]
    return "|".join(names) if names else "NONE"


# ---------------------------------------------------------------------------
# sensor_structure.c mirror — packing / unpacking
#
# The C structs in tools/sensor_structure.c are __attribute__((packed)) and use
# fixed-width integer fields (no bare enums), so the on-wire layout is a single,
# deterministic, little-endian byte sequence — independent of the compiler's
# alignment rules or -fshort-enums. The formats below mirror that exactly.
# ---------------------------------------------------------------------------

SENSOR_REPORT_INFO_MAX = 16
SENSOR_CONFIG_INFO_MAX = 16
SENSOR_LARGE_DATA_INFO_MAX = 200 * 1024   # whole sensor_large_data_structure_t budget (200 KB)
SENSOR_DATA_STR_SN_MAX = 12
SENSOR_REPORT_CONFIG_NAME_MAX = 6   # "ESC33"/"ESC21" fit (5 chars + NUL)

# sensor_alarm_flags_t bits.
ALARM_NONE         = 0x0000
ALARM_BATTERY      = 1 << 0
ALARM_TEMPERATURE1 = 1 << 1
ALARM_TEMPERATURE2 = 1 << 2
ALARM_HUMIDITY1    = 1 << 3
ALARM_HUMIDITY2    = 1 << 4
ALARM_ULTRASOUND   = 1 << 9
ALARM_VIBRATION    = 1 << 10

# sensor_config_cmd_t bits.
CONFIG_CMD_FLAGS = {
    "LOGS": 1 << 0,
    "LOGS_CLEAR": 1 << 1,
    "DEMO_MODE": 1 << 2,
    "RESET": 1 << 5,
    "DELETE_CERTIFICATE": 1 << 6,
    "OTA": 1 << 7,
}
# Commands this simulator acts on, per the spec (no, demo_mode, reset).
CMD_DEMO_MODE = CONFIG_CMD_FLAGS["DEMO_MODE"]
CMD_RESET     = CONFIG_CMD_FLAGS["RESET"]


def _name_bytes(s: str, n: int) -> bytes:
    b = s.encode("ascii", errors="replace")[:n]
    return b + b"\x00" * (n - len(b))


def _report_fmt() -> str:
    """struct format for sensor_data_structure_t (packed, fixed-width)."""
    return ("<"
            "6s"   # name[6]
            "12s"  # sn[12]
            "H"    # report_type   (sensor_report_type_t)
            "H"    # firmware_version
            "B"    # battery_level
            "H"    # alarm_flags    (sensor_alarm_flags_t bitmask)
            "B"    # report_flags   (sensor_report_config_flags_t bitmask)
            "H"    # report_crc16
            f"{SENSOR_REPORT_INFO_MAX}s")  # report_info union


def _config_fmt() -> str:
    """struct format for sensor_config_structure_t (packed, fixed-width)."""
    return ("<"
            "6s"   # name[6]
            "6s"   # dest_id[6]
            "B"    # command        (sensor_config_cmd_t bitmask)
            "H"    # new_firmware_version
            "B"    # battery_level_min
            "H"    # sleep_time_sec
            "B"    # config_flags   (sensor_report_config_flags_t bitmask)
            "H"    # config_crc16
            f"{SENSOR_CONFIG_INFO_MAX}s")  # config_info union


# ---------------------------------------------------------------------------
# Per-type report_info / config_info layouts (sensor_report_info_<type>_t and
# sensor_config_info_<type>_t in sensor_structure.c). Every struct sits inside
# a #pragma pack(push, 1) block, so the layout is fully packed (no internal
# padding) and simply zero-filled out to the 16-byte union.
#
# Each field is (name, struct_code, scale): scale 100 means the raw integer is
# hundredths (temperatures °C / humidities %RH); scale 1 is a raw count.
# ---------------------------------------------------------------------------

# Reusable report_info fields.
_R_T1 = ("temperature1", "h", 100)
_R_H1 = ("humidity1", "H", 100)
_R_T2 = ("temperature2", "h", 100)
_R_H2 = ("humidity2", "H", 100)
_R_UL = ("ultrasound_level", "H", 1)
_R_UF = ("ultrasound_frequency", "B", 1)
_R_VL = ("vibration_level", "H", 1)
_R_VF = ("vibration_frequency", "B", 1)

REPORT_INFO_LAYOUT = {
    0x3100: [_R_T1, _R_H1, _R_VL, _R_VF],
    0x3101: [_R_T1, _R_H1, _R_T2, _R_H2, _R_VL, _R_VF],
    0x3102: [_R_T1, _R_H1, ("gas1", "H", 1), ("gas2", "H", 1), ("gas3", "H", 1),
             _R_VL, _R_VF],
    0x3103: [_R_T1, _R_H1, _R_T2, _R_H2, _R_VL, _R_VF],
    0x3104: [_R_T1, _R_H1, _R_T2, _R_H2, ("current", "H", 1), _R_VL, _R_VF],
    0x3105: [_R_T1, _R_H1, _R_T2, _R_H2, _R_UL, _R_UF, _R_VL, _R_VF],
    0x3200: [_R_T1, _R_H1, _R_UL, _R_UF],
    0x3300: [_R_T1, _R_H1, _R_T2, _R_H2, _R_UL, _R_UF, _R_VL, _R_VF],
}

# Reusable config_info fields.
_C_TMX1 = ("temperature_max1", "b", 1)
_C_TMN1 = ("temperature_min1", "b", 1)
_C_HMX1 = ("humidity_max1", "B", 1)
_C_HMN1 = ("humidity_min1", "B", 1)
_C_TMX2 = ("temperature_max2", "b", 1)
_C_TMN2 = ("temperature_min2", "b", 1)
_C_HMX2 = ("humidity_max2", "B", 1)
_C_HMN2 = ("humidity_min2", "B", 1)
_C_ULMX = ("ultrasound_level_max", "H", 1)
_C_UCF = ("ultrasound_center_frequency", "B", 1)
_C_VMX = ("vibration_level_max", "H", 1)
_C_RND = ("random_number", "B", 1)

CONFIG_INFO_LAYOUT = {
    0x3100: [_C_TMX1, _C_TMN1, _C_HMX1, _C_HMN1, _C_VMX, _C_RND],
    0x3101: [_C_TMX1, _C_TMN1, _C_HMX1, _C_HMN1, _C_TMX2, _C_TMN2, _C_HMX2, _C_HMN2,
             _C_VMX, _C_RND],
    0x3102: [_C_TMX1, _C_TMN1, _C_HMX1, _C_HMN1, ("gas1_max", "H", 1),
             ("gas2_max", "H", 1), ("gas3_max", "H", 1), _C_VMX, _C_RND],
    0x3103: [_C_TMX1, _C_TMN1, _C_HMX1, _C_HMN1, _C_TMX2, _C_TMN2, _C_HMX2, _C_HMN2,
             _C_VMX, _C_RND],
    0x3104: [_C_TMX1, _C_TMN1, _C_HMX1, _C_HMN1, _C_TMX2, _C_TMN2, _C_HMX2, _C_HMN2,
             ("current_max", "H", 1), ("current_min", "H", 1), _C_VMX, _C_RND],
    0x3105: [_C_TMX1, _C_TMN1, _C_HMX1, _C_HMN1, _C_TMX2, _C_TMN2, _C_HMX2, _C_HMN2,
             _C_ULMX, _C_UCF, _C_VMX, _C_RND],
    0x3200: [_C_TMX1, _C_TMN1, _C_HMX1, _C_HMN1, _C_ULMX, _C_UCF, _C_RND],
    0x3300: [_C_TMX1, _C_TMN1, _C_HMX1, _C_HMN1, _C_TMX2, _C_TMN2, _C_HMX2, _C_HMN2,
             _C_ULMX, _C_UCF, _C_VMX, _C_RND],
}


def _info_fmt(layout) -> str:
    """Little-endian, fully packed struct format for an info layout."""
    return "<" + "".join(code for _, code, _ in layout)


def _decode_info(layout, data: bytes) -> dict:
    """Unpack the leading bytes of an info union per `layout`, applying scales."""
    fmt = _info_fmt(layout)
    vals = struct.unpack(fmt, data[:struct.calcsize(fmt)])
    out = {}
    for (name, _code, scale), v in zip(layout, vals):
        out[name] = (v / 100.0) if scale == 100 else v
    return out


def _report_info_3300_fmt() -> str:
    """Packed struct format for sensor_report_info_3300_t (the 14-byte 3300
    report_info; zero-padded out to the 16-byte report_info union)."""
    return _info_fmt(REPORT_INFO_LAYOUT[0x3300])


def _config_info_3300_fmt() -> str:
    """Packed struct format for sensor_config_info_3300_t (14 bytes, zero-padded
    out to the 16-byte config_info union)."""
    return _info_fmt(CONFIG_INFO_LAYOUT[0x3300])


def _crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    """CRC16/CCITT-FALSE — fills the in-struct report_crc16/config_crc16 field
    so the structure is self-consistent. The AT framing's CRC32 is what the
    firmware actually verifies."""
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def _temp_x100(c: float) -> int:
    """°C -> int16 hundredths of a degree, clamped."""
    return max(-32768, min(32767, int(round(c * 100))))


def _hum_x100(p: float) -> int:
    """%RH -> uint16 hundredths of a percent, clamped."""
    return max(0, min(65535, int(round(p * 100))))


def build_report_info_3300(*, temperature1: float, humidity1: float,
                           temperature2: float = 0.0, humidity2: float = 0.0,
                           ultrasound_level: int = 0, ultrasound_frequency: int = 0,
                           vibration_level: int = 0, vibration_frequency: int = 0) -> bytes:
    """Pack sensor_report_info_3300_t into the 16-byte report_info union.

    Temperatures are int16 ×100 °C, humidities uint16 ×100 %; ultrasound /
    vibration levels are raw uint16 and their frequencies raw uint8. Zero-padded
    out to SENSOR_REPORT_INFO_MAX."""
    info = struct.pack(
        _report_info_3300_fmt(),
        _temp_x100(temperature1), _hum_x100(humidity1),
        _temp_x100(temperature2), _hum_x100(humidity2),
        ultrasound_level & 0xFFFF, ultrasound_frequency & 0xFF,
        vibration_level & 0xFFFF, vibration_frequency & 0xFF,
    )
    return info + b"\x00" * (SENSOR_REPORT_INFO_MAX - len(info))


def build_report_payload(*, sn_hex: str, battery_level: int, alarm_flags: int,
                         report_info: bytes, firmware_version: int,
                         report_flags: int = FLAG_NOT_ENCRYPTED) -> bytes:
    """Pack a sensor_data_structure_t (report) into its on-wire bytes."""
    if len(report_info) != SENSOR_REPORT_INFO_MAX:
        raise ValueError(f"report_info must be {SENSOR_REPORT_INFO_MAX} bytes")
    report_crc16 = _crc16_ccitt(report_info)
    return struct.pack(
        _report_fmt(),
        _name_bytes(REPORT_NAME, SENSOR_REPORT_CONFIG_NAME_MAX),
        _name_bytes(sn_hex.upper(), SENSOR_DATA_STR_SN_MAX),
        REPORT_TYPE_3300,
        firmware_version & 0xFFFF,
        battery_level & 0xFF,
        alarm_flags & 0xFFFF,      # alarm_flags is uint16_t
        report_flags & 0xFF,       # report_flags is uint8_t
        report_crc16 & 0xFFFF,
        report_info,
    )


def decode_report_payload(payload: bytes) -> Optional[dict]:
    """Decode a sensor_data_structure_t from AT#REPORT <DATA>. Returns a dict
    with a 'raw' marker on length mismatch so the caller can still show it."""
    fmt = _report_fmt()
    want = struct.calcsize(fmt)
    if len(payload) != want:
        return {"raw": payload.hex().upper(),
                "note": f"len {len(payload)} != expected {want}"}
    (name, sn, rtype, fw, battery, alarm_flags,
     report_flags, report_crc16, report_info) = struct.unpack(fmt, payload)
    # Decode the report_info union per the report_type (see REPORT_INFO_LAYOUT).
    layout = REPORT_INFO_LAYOUT.get(rtype)
    info = {}
    if layout is not None:
        try:
            info = _decode_info(layout, report_info)
        except struct.error:
            info = {}
    result = {
        "name": name.split(b"\x00", 1)[0].decode("ascii", "replace"),
        "sn": sn.split(b"\x00", 1)[0].decode("ascii", "replace"),
        "report_type": rtype,
        "report_type_known": layout is not None,
        "firmware_version": fw,
        "battery_level": battery,
        "alarm_flags_raw": alarm_flags,
        "alarm_flags": f"0x{alarm_flags:04X} ({alarm_text(alarm_flags)})",
        "report_flags_raw": report_flags,
        "report_flags": f"0x{report_flags:02X}",
        "report_crc16": f"0x{report_crc16:04X}",
        # All decoded type-specific fields (e.g. gas1/gas2/gas3, current).
        "report_info": info,
        "report_info_hex": report_info.hex().upper(),
        # Canonical columns default to 0 so consumers always have the keys,
        # regardless of which fields this report_type actually carries.
        "temperature1": info.get("temperature1", 0.0),
        "humidity1": info.get("humidity1", 0.0),
        "temperature2": info.get("temperature2", 0.0),
        "humidity2": info.get("humidity2", 0.0),
        "ultrasound_level": info.get("ultrasound_level", 0),
        "ultrasound_frequency": info.get("ultrasound_frequency", 0),
        "vibration_level": info.get("vibration_level", 0),
        "vibration_frequency": info.get("vibration_frequency", 0),
    }
    # channel-1 temp/hum kept as "temperature"/"humidity" for back-compat.
    result["temperature"] = result["temperature1"]
    result["humidity"] = result["humidity1"]
    # Surface type-specific extras at the top level for convenience.
    for k in ("gas1", "gas2", "gas3", "current"):
        if k in info:
            result[k] = info[k]
    return result


def decode_config_payload(payload: bytes,
                          sensor_type: int = 0x3300) -> Optional[dict]:
    """Decode a sensor_config_structure_t from AT#CONFIG <DATA>. Returns a dict
    with a 'raw' marker on length mismatch so the caller can still show it.

    The config header carries no sensor type, so `sensor_type` selects which
    sensor_config_info_<type>_t layout to apply to the config_info union
    (defaults to 0x3300). See CONFIG_INFO_LAYOUT."""
    fmt = _config_fmt()
    want = struct.calcsize(fmt)
    if len(payload) != want:
        return {"raw": payload.hex().upper(),
                "note": f"len {len(payload)} != expected {want}"}
    (name, dest_id, command, new_fw, bat_min, sleep_s,
     config_flags, config_crc16, config_info) = struct.unpack(fmt, payload)
    cmds = [k for k, v in CONFIG_CMD_FLAGS.items() if command & v]
    # Decode the config_info union per the assumed sensor_type.
    layout = CONFIG_INFO_LAYOUT.get(sensor_type)
    info = {}
    if layout is not None:
        try:
            info = _decode_info(layout, config_info)
        except struct.error:
            info = {}
    return {
        "name": name.split(b"\x00", 1)[0].decode("ascii", "replace"),
        "dest_id": dest_id.hex().upper(),
        "command_raw": command,
        "command": f"0x{command:02X} ({'|'.join(cmds) if cmds else 'NONE'})",
        "new_firmware_version": new_fw,
        "battery_level_min": bat_min,
        "sleep_time_sec": sleep_s,
        "config_flags_raw": config_flags,
        "config_flags": f"0x{config_flags:02X}",
        "config_crc16": f"0x{config_crc16:04X}",
        "config_info": config_info.hex().upper(),
        "sensor_type": sensor_type,
        "sensor_type_known": layout is not None,
        # Decoded config_info fields for this sensor_type.
        "config_info_decoded": info,
        # Back-compat alias kept for 3300 consumers (apply_config et al.).
        "config_info_3300": info,
    }


def build_config_payload(*, dest_sn_hex: str, command: int, new_fw_version: int,
                         battery_level_min: int, sleep_time_sec: int,
                         temp_max: int, temp_min: int, hum_max: int, hum_min: int,
                         temp_max2: int = 0, temp_min2: int = 0,
                         hum_max2: int = 0, hum_min2: int = 0,
                         ultrasound_level_max: int = 0,
                         ultrasound_center_frequency: int = 0,
                         vibration_level_max: int = 0,
                         random_number: int = 0,
                         config_flags: int = FLAG_NOT_ENCRYPTED) -> bytes:
    """Pack a sensor_config_structure_t (name "ESC21") for an AT#CONFIG push.

    dest_sn_hex is the destination sensor SN as hex; it fills dest_id[6] as raw
    bytes (big-endian, last 6 bytes of the SN). config_info holds a
    sensor_config_info_3300_t: temp/hum max/min for channels 1 and 2 (temp_max..
    /temp_max2..), ultrasound + vibration level maxima, and a random byte.
    The channel-1 args keep their original names for back-compat."""
    dest = int(dest_sn_hex, 16).to_bytes(8, "big")[-6:]   # low 6 bytes of SN
    info = struct.pack(
        _config_info_3300_fmt(),
        temp_max, temp_min, hum_max & 0xFF, hum_min & 0xFF,
        temp_max2, temp_min2, hum_max2 & 0xFF, hum_min2 & 0xFF,
        ultrasound_level_max & 0xFFFF, ultrasound_center_frequency & 0xFF,
        vibration_level_max & 0xFFFF, random_number & 0xFF,
    )
    info += b"\x00" * (SENSOR_CONFIG_INFO_MAX - len(info))
    config_crc16 = _crc16_ccitt(info)
    return struct.pack(
        _config_fmt(),
        _name_bytes(CONFIG_NAME, SENSOR_REPORT_CONFIG_NAME_MAX),
        dest,
        command & 0xFF,            # command is uint8_t
        new_fw_version & 0xFFFF,
        battery_level_min & 0xFF,
        sleep_time_sec & 0xFFFF,
        config_flags & 0xFF,       # config_flags is uint8_t
        config_crc16 & 0xFFFF,
        info,
    )


# ---------------------------------------------------------------------------
# sensor_large_data_structure_t (name "ESC33", data_type 0x3300)
#
# Mirrors sensor_structure.c: a 29-byte fixed header followed by the data_info
# union. For data_type 0x3300 the union is a sensor_large_data_info_3300_t — a
# 14-byte sensor_report_info_3300_t header immediately followed by the raw
# sound_record samples. The WHOLE structure is bounded by
# SENSOR_LARGE_DATA_INFO_MAX (200 KB): header(29) + union(SENSOR_LARGE_DATA_INFO_MAX
# - 29) == 200 KB. In firmware it's staged in PSRAM; on the wire we serialize
# only the populated data_info bytes, so the transferred blob is header + data_info.
# ---------------------------------------------------------------------------

def _large_data_fmt() -> str:
    """struct format for the 29-byte fixed header of
    sensor_large_data_structure_t (packed, fixed-width)."""
    return ("<"
            "6s"   # name[6]
            "12s"  # sn[12]
            "H"    # data_type        (sensor_large_data_type_t)
            "H"    # firmware_version
            "B"    # battery_level
            "I"    # total_size       (size of the populated data_info)
            "H")   # large_data_crc16


LARGE_DATA_HEADER_SIZE = struct.calcsize(_large_data_fmt())        # 29
REPORT_INFO_3300_SIZE = struct.calcsize(_report_info_3300_fmt())   # 14

# Largest sound_record, derived so the WHOLE structure stays <= 200 KB:
# header(29) + report_info_3300(14) + sound_record <= SENSOR_LARGE_DATA_INFO_MAX.
# This matches the C sound_record[SENSOR_LARGE_DATA_INFO_MAX - 14 - 29] array.
LARGE_DATA_SOUND_RECORD_MAX = (SENSOR_LARGE_DATA_INFO_MAX
                               - LARGE_DATA_HEADER_SIZE - REPORT_INFO_3300_SIZE)
# A single mesh transfer is also capped at LARGE_DATA_MAX_TRANSFER; the 200 KB
# structure fits because both budgets are 200 KB.
assert SENSOR_LARGE_DATA_INFO_MAX <= LARGE_DATA_MAX_TRANSFER, \
    "large-data structure budget exceeds the mesh transfer cap"


def build_large_data_info_3300(sound_record: bytes, *, temperature1: float,
                               humidity1: float, temperature2: float = 0.0,
                               humidity2: float = 0.0, ultrasound_level: int = 0,
                               ultrasound_frequency: int = 0,
                               vibration_level: int = 0,
                               vibration_frequency: int = 0) -> bytes:
    """Pack a sensor_large_data_info_3300_t: the 14-byte sensor_report_info_3300_t
    header (same field layout as the 3300 report_info) followed by the raw
    sound_record samples. Unlike build_report_info_3300() this is NOT padded out
    to a fixed union — only the populated data_info bytes are serialized, and the
    sound_record is capped (LARGE_DATA_SOUND_RECORD_MAX) so the whole wrapped
    structure stays <= SENSOR_LARGE_DATA_INFO_MAX (200 KB)."""
    info = struct.pack(
        _report_info_3300_fmt(),
        _temp_x100(temperature1), _hum_x100(humidity1),
        _temp_x100(temperature2), _hum_x100(humidity2),
        ultrasound_level & 0xFFFF, ultrasound_frequency & 0xFF,
        vibration_level & 0xFFFF, vibration_frequency & 0xFF,
    )
    return info + bytes(sound_record)


def build_large_data_payload(*, sn_hex: str, firmware_version: int,
                             battery_level: int, data_info: bytes,
                             data_type: int = LARGE_DATA_TYPE_3300) -> bytes:
    """Pack a sensor_large_data_structure_t into its on-wire bytes: the 29-byte
    header (name "ESC33", SN, data_type, firmware, battery, total_size, crc16)
    followed by data_info. total_size records the populated data_info length;
    large_data_crc16 is CCITT-FALSE over data_info, matching the report/config
    convention in this file."""
    total_size = len(data_info)
    large_data_crc16 = _crc16_ccitt(data_info)
    header = struct.pack(
        _large_data_fmt(),
        _name_bytes(LARGE_DATA_NAME, SENSOR_REPORT_CONFIG_NAME_MAX),
        _name_bytes(sn_hex.upper(), SENSOR_DATA_STR_SN_MAX),
        data_type & 0xFFFF,
        firmware_version & 0xFFFF,
        battery_level & 0xFF,
        total_size & 0xFFFFFFFF,
        large_data_crc16 & 0xFFFF,
    )
    return header + data_info


def decode_large_data_payload(payload: bytes) -> Optional[dict]:
    """Decode a reassembled sensor_large_data_structure_t blob (the bytes a
    large-data transfer carries). Returns a dict with a 'raw' marker if the blob
    is too short to even hold the 29-byte header. For data_type 0x3300 the
    data_info_3300 (a 14-byte sensor_report_info_3300_t + sound_record) is
    decoded; the sound_record length is reported but the samples are not."""
    hdr_fmt = _large_data_fmt()
    hdr_size = struct.calcsize(hdr_fmt)
    if len(payload) < hdr_size:
        return {"raw": payload.hex().upper(),
                "note": f"len {len(payload)} < header {hdr_size}"}
    (name, sn, data_type, fw, battery,
     total_size, large_data_crc16) = struct.unpack(hdr_fmt, payload[:hdr_size])
    data_info = payload[hdr_size:]
    info = {}
    sound_record_len = 0
    known = (data_type == LARGE_DATA_TYPE_3300)
    if known:
        layout = REPORT_INFO_LAYOUT[0x3300]
        ri_size = struct.calcsize(_info_fmt(layout))   # 14
        if len(data_info) >= ri_size:
            try:
                info = _decode_info(layout, data_info[:ri_size])
            except struct.error:
                info = {}
            sound_record_len = len(data_info) - ri_size
    return {
        "name": name.split(b"\x00", 1)[0].decode("ascii", "replace"),
        "sn": sn.split(b"\x00", 1)[0].decode("ascii", "replace"),
        "data_type": data_type,
        "data_type_known": known,
        "firmware_version": fw,
        "battery_level": battery,
        # total_size is the populated data_info length stored in the header.
        "total_size": total_size,
        "large_data_crc16": f"0x{large_data_crc16:04X}",
        "data_info_len": len(data_info),
        "sound_record_len": sound_record_len,
        "report_info": info,
        "temperature1": info.get("temperature1", 0.0),
        "humidity1": info.get("humidity1", 0.0),
        "temperature2": info.get("temperature2", 0.0),
        "humidity2": info.get("humidity2", 0.0),
        "ultrasound_level": info.get("ultrasound_level", 0),
        "ultrasound_frequency": info.get("ultrasound_frequency", 0),
        "vibration_level": info.get("vibration_level", 0),
        "vibration_frequency": info.get("vibration_frequency", 0),
    }


# ---------------------------------------------------------------------------
# AT framing
# ---------------------------------------------------------------------------

def _quoted_fields(line: str) -> Optional[list[str]]:
    out: list[str] = []
    i, n = 0, len(line)
    while i < n:
        a = line.find('"', i)
        if a < 0:
            break
        b = line.find('"', a + 1)
        if b < 0:
            return None
        out.append(line[a + 1 : b])
        i = b + 1
    return out


def build_at_report(sn: int, data_id: int, payload: bytes, priority: int) -> str:
    """AT#REPORT="<SN16>","<ID4>","<LEN4>","<PRIO2>","<CRC8>","<HEX>" — 6 fields,
    matching cmd_report() in src/slm_at_main.c."""
    if not (0 < len(payload) <= MAX_REPORT_SIZE):
        raise ValueError(f"payload must be 1..{MAX_REPORT_SIZE} bytes")
    crc32 = zlib.crc32(payload) & 0xFFFFFFFF
    return (
        '\r\nAT#REPORT='
        f'"{sn:016X}","{data_id:04X}","{len(payload):04X}",'
        f'"{priority:02X}","{crc32:08X}","{payload.hex().upper()}"\r\n'
    )


def build_at_config(sn: int, data_id: int, payload: bytes) -> str:
    """AT#CONFIG="<SN16>","<ID4>","<LEN4>","<CRC8>","<HEX>" — 5 fields, matching
    cmd_config() in src/slm_at_main.c (the gateway-only config push)."""
    if not (0 < len(payload) <= MAX_CONFIG_SIZE):
        raise ValueError(f"payload must be 1..{MAX_CONFIG_SIZE} bytes")
    crc32 = zlib.crc32(payload) & 0xFFFFFFFF
    return (
        '\r\nAT#CONFIG='
        f'"{sn:016X}","{data_id:04X}","{len(payload):04X}",'
        f'"{crc32:08X}","{payload.hex().upper()}"\r\n'
    )


def build_at_ldinit(sn: int, data_id: int, total_size: int,
                    total_chunks: int, last_chunk_size: int,
                    data_crc32: int) -> str:
    """AT#LDINIT="<SN16>","<ID4>","<TOTAL_SIZE8>","<TOTAL_CHUNKS4>","<LAST_CHUNK_SIZE4>","<CRC32_8>","<CRC16_4>"
    — 7 fields. Announces a large-data transfer: total_size is the size of the
    whole structure being sent; total_chunks/last_chunk_size describe the AT#LD
    framing.

    There are two distinct CRCs here, covering two different things:
      * CRC32 = IEEE (zlib.crc32) over the ENTIRE large-data blob. The device
        stores it and, once every AT#LD chunk has arrived, recomputes the CRC32
        over the staged blob to verify the whole transfer end-to-end (and to
        forward it upstream over the mesh).
      * CRC16 = CCITT-FALSE over the init metadata itself (this line's fields,
        NOT the data) so the device can verify the init line arrived intact
        over serial. It now also covers the CRC32 field, so a corrupted
        whole-data CRC32 is caught here too.

    CRC16 is taken over the metadata packed big-endian as
    <SN:u64><ID:u16><TOTAL_SIZE:u32><TOTAL_CHUNKS:u16><LAST_CHUNK_SIZE:u16><CRC32:u32>,
    matching crc16(0x1021, 0xFFFF, ...) in src/large_data.c cmd_ld_init()."""
    if not (0 < total_size <= LARGE_DATA_MAX_TRANSFER):
        raise ValueError(f"total_size must be 1..{LARGE_DATA_MAX_TRANSFER} bytes")
    if not (0 < total_chunks <= 0xFFFF):
        raise ValueError("total_chunks must be 1..0xFFFF")
    if not (0 < last_chunk_size <= AT_LD_PAYLOAD_MAX):
        raise ValueError(f"last_chunk_size must be 1..{AT_LD_PAYLOAD_MAX}")
    meta = struct.pack(">QHIHHI", sn & 0xFFFFFFFFFFFFFFFF, data_id & 0xFFFF,
                       total_size & 0xFFFFFFFF, total_chunks & 0xFFFF,
                       last_chunk_size & 0xFFFF, data_crc32 & 0xFFFFFFFF)
    crc16 = _crc16_ccitt(meta)
    return (
        '\r\nAT#LDINIT='
        f'"{sn:016X}","{data_id:04X}","{total_size:08X}",'
        f'"{total_chunks:04X}","{last_chunk_size:04X}",'
        f'"{data_crc32 & 0xFFFFFFFF:08X}","{crc16:04X}"\r\n'
    )


def build_at_ld(sn: int, data_id: int, page: int, chunk: int, payload: bytes) -> str:
    """AT#LD="<SN16>","<ID4>","<PAGE2>","<CHUNK2>","<CRC16_4>","<HEX>" — 6 fields,
    one serial chunk of the large-data transfer announced by AT#LDINIT (matched
    by the same SN + ID). CRC16 is CCITT-FALSE over the chunk payload. Payload is
    hex-encoded and capped at AT_LD_PAYLOAD_MAX so the framed line fits the
    firmware's 1024-byte SLM_UART_AT_COMMAND_LEN buffer."""
    if not (0 < len(payload) <= AT_LD_PAYLOAD_MAX):
        raise ValueError(f"payload must be 1..{AT_LD_PAYLOAD_MAX} bytes")
    if not (0 <= data_id <= 0xFFFF):
        raise ValueError("data_id must fit uint16 (0..0xFFFF)")
    if not (0 <= page <= 0xFF) or not (0 <= chunk <= 0xFF):
        raise ValueError("page/chunk must fit uint8 (0..255)")
    crc16 = _crc16_ccitt(payload)
    return (
        '\r\nAT#LD='
        f'"{sn:016X}","{data_id:04X}","{page:02X}","{chunk:02X}",'
        f'"{crc16:04X}","{payload.hex().upper()}"\r\n'
    )


def iter_ld_chunks(blob: bytes, chunk_size: int = AT_LD_PAYLOAD_MAX):
    """Yield (page, chunk, payload) splitting blob into <=chunk_size serial
    chunks, indexed page/chunk (both uint8; chunk wraps every LD_CHUNKS_PER_PAGE,
    incrementing page). Raises if the transfer needs more than 256 pages."""
    n = (len(blob) + chunk_size - 1) // chunk_size
    for i in range(n):
        page, chunk = divmod(i, LD_CHUNKS_PER_PAGE)
        if page > 0xFF:
            raise ValueError(
                f"transfer needs {n} chunks, exceeds {0x100 * LD_CHUNKS_PER_PAGE} "
                f"addressable with uint8 page/chunk")
        yield page, chunk, blob[i * chunk_size:(i + 1) * chunk_size]


def parse_at_report(line: str) -> tuple[bool, str, Optional[dict]]:
    """Parse + CRC-verify an incoming AT#REPORT as seen on the GATEWAY side:
    6 fields, NO priority but WITH a report timestamp —
        sn, id, timestamp(uint64), len, crc32, payload
    matching the gateway emission in src/data.c data_tick(). (The sensor-side
    AT#REPORT that build_at_report() emits instead carries a priority field;
    the gateway strips priority and stamps a timestamp before forwarding.)

    Returns (ok, summary, parsed); parsed has 'timestamp' (ms uptime from the
    gateway, k_uptime_get) and 'priority'=None for this path."""
    fields = _quoted_fields(line)
    if fields is None or len(fields) < 6:
        return False, f"malformed: expected 6 quoted fields, got {fields!r}", None
    sn_hex, id_hex, ts_hex, len_hex, crc_hex, data_hex = fields[:6]
    try:
        sn = int(sn_hex, 16); data_id = int(id_hex, 16)
        timestamp = int(ts_hex, 16)
        data_len = int(len_hex, 16); crc32 = int(crc_hex, 16)
    except ValueError as e:
        return False, f"bad hex in header: {e}", None
    if len(data_hex) != data_len * 2:
        return False, f"len mismatch: header {data_len} B, got {len(data_hex)} hex chars", None
    try:
        payload = bytes.fromhex(data_hex)
    except ValueError as e:
        return False, f"bad hex in payload: {e}", None
    calc = zlib.crc32(payload) & 0xFFFFFFFF
    if calc != crc32:
        return False, f"CRC32 mismatch: header 0x{crc32:08X}, calc 0x{calc:08X}", None
    summary = f"sn=0x{sn:016X} id={data_id} ts={timestamp}ms len={data_len} crc=0x{crc32:08X}"
    return True, summary, {"sn": sn, "data_id": data_id, "timestamp": timestamp,
                           "data_len": data_len, "priority": None,
                           "crc32": crc32, "payload": payload}


def parse_at_config(line: str) -> tuple[bool, str, Optional[dict]]:
    fields = _quoted_fields(line)
    if fields is None or len(fields) < 5:
        return False, f"malformed: expected 5 quoted fields, got {fields!r}", None
    sn_hex, id_hex, len_hex, crc_hex, data_hex = fields[:5]
    try:
        sn = int(sn_hex, 16); data_id = int(id_hex, 16)
        data_len = int(len_hex, 16); crc32 = int(crc_hex, 16)
    except ValueError as e:
        return False, f"bad hex in header: {e}", None
    if len(data_hex) != data_len * 2:
        return False, f"len mismatch: header {data_len} B, got {len(data_hex)} hex chars", None
    try:
        payload = bytes.fromhex(data_hex)
    except ValueError as e:
        return False, f"bad hex in payload: {e}", None
    calc = zlib.crc32(payload) & 0xFFFFFFFF
    if calc != crc32:
        return False, f"CRC32 mismatch: header 0x{crc32:08X}, calc 0x{calc:08X}", None
    summary = f"sn=0x{sn:016X} id={data_id} len={data_len} crc=0x{crc32:08X}"
    return True, summary, {"sn": sn, "data_id": data_id, "data_len": data_len,
                           "crc32": crc32, "payload": payload}


def parse_at_ldinit(line: str) -> tuple[bool, str, Optional[dict]]:
    """Parse + CRC16-verify an incoming AT#LDINIT (the 7-field large-data init,
    same framing build_at_ldinit() emits). Verifies the CCITT-FALSE CRC16 over
    the init metadata (SN/ID/total_size/total_chunks/last_chunk_size/CRC32),
    matching cmd_ld_init() in src/large_data.c. Returns (ok, summary, parsed)
    where parsed carries the whole-blob CRC32 for end-to-end verification."""
    fields = _quoted_fields(line)
    if fields is None or len(fields) < 7:
        return False, f"malformed: expected 7 quoted fields, got {fields!r}", None
    sn_hex, id_hex, total_hex, chunks_hex, last_hex, crc32_hex, crc16_hex = fields[:7]
    try:
        sn = int(sn_hex, 16); data_id = int(id_hex, 16)
        total_size = int(total_hex, 16); total_chunks = int(chunks_hex, 16)
        last_chunk_size = int(last_hex, 16); crc32 = int(crc32_hex, 16)
        crc16 = int(crc16_hex, 16)
    except ValueError as e:
        return False, f"bad hex in header: {e}", None
    meta = struct.pack(">QHIHHI", sn & 0xFFFFFFFFFFFFFFFF, data_id & 0xFFFF,
                       total_size & 0xFFFFFFFF, total_chunks & 0xFFFF,
                       last_chunk_size & 0xFFFF, crc32 & 0xFFFFFFFF)
    calc = _crc16_ccitt(meta)
    if calc != crc16:
        return False, f"CRC16 mismatch: header 0x{crc16:04X}, calc 0x{calc:04X}", None
    summary = (f"sn=0x{sn:016X} id={data_id} total={total_size} "
               f"chunks={total_chunks} last={last_chunk_size} crc32=0x{crc32:08X}")
    return True, summary, {"sn": sn, "data_id": data_id, "total_size": total_size,
                           "total_chunks": total_chunks,
                           "last_chunk_size": last_chunk_size,
                           "crc32": crc32, "crc16": crc16}


def parse_at_ld(line: str) -> tuple[bool, str, Optional[dict]]:
    """Parse + CRC16-verify one incoming AT#LD chunk (the 6-field framing
    build_at_ld() emits). Verifies the CCITT-FALSE CRC16 over the chunk payload.
    Returns (ok, summary, parsed) with the decoded page/chunk indices and the
    raw payload bytes."""
    fields = _quoted_fields(line)
    if fields is None or len(fields) < 6:
        return False, f"malformed: expected 6 quoted fields, got {fields!r}", None
    sn_hex, id_hex, page_hex, chunk_hex, crc16_hex, data_hex = fields[:6]
    try:
        sn = int(sn_hex, 16); data_id = int(id_hex, 16)
        page = int(page_hex, 16); chunk = int(chunk_hex, 16)
        crc16 = int(crc16_hex, 16)
    except ValueError as e:
        return False, f"bad hex in header: {e}", None
    try:
        payload = bytes.fromhex(data_hex)
    except ValueError as e:
        return False, f"bad hex in payload: {e}", None
    calc = _crc16_ccitt(payload)
    if calc != crc16:
        return False, f"CRC16 mismatch: header 0x{crc16:04X}, calc 0x{calc:04X}", None
    # Linear chunk index, matching iter_ld_chunks()'s page/chunk pagination.
    linear = page * LD_CHUNKS_PER_PAGE + chunk
    summary = (f"sn=0x{sn:016X} id={data_id} page={page} chunk={chunk} "
               f"linear={linear} len={len(payload)}")
    return True, summary, {"sn": sn, "data_id": data_id, "page": page,
                           "chunk": chunk, "linear": linear,
                           "crc16": crc16, "payload": payload}


# ---------------------------------------------------------------------------
# Sensor state model
# ---------------------------------------------------------------------------

class SensorState:
    """Mutable sensor state, guarded by a lock since the config reader thread
    and the main loop both touch it."""

    def __init__(self, sn: int, battery: int, fw_version: int,
                 sleep_time_sec: int, battery_level_min: int):
        self.lock = threading.Lock()
        self.sn = sn
        self.battery = battery                  # 0..100
        self.fw_version = fw_version
        self.sleep_time_sec = sleep_time_sec    # report cadence
        self.battery_level_min = battery_level_min
        self.demo_mode = False
        # config_info_3300 thresholds (None until a 3300 config arrives).
        # Channel 1 keeps the original temp_max/.. names; channel 2 + ultrasound /
        # vibration maxima are the new sensor_config_info_3300_t fields.
        self.temp_max = None                    # int8 °C  (channel 1)
        self.temp_min = None
        self.hum_max = None                     # uint8 %  (channel 1)
        self.hum_min = None
        self.temp_max2 = None                   # int8 °C  (channel 2)
        self.temp_min2 = None
        self.hum_max2 = None                    # uint8 %  (channel 2)
        self.hum_min2 = None
        self.ultrasound_level_max = None        # uint16 raw
        self.ultrasound_center_frequency = None  # uint8 raw
        self.vibration_level_max = None         # uint16 raw
        self.config_flags = FLAG_NOT_ENCRYPTED
        self.last_config_ts = None              # wall-clock str of last config
        # Buffered readings (regenerated every GEN_PERIOD_S, sent on report tick).
        # temperature/humidity are channel 1; the rest are the new 3300 fields.
        self.temperature = 22.0
        self.humidity = 45.0
        self.temperature2 = 22.0
        self.humidity2 = 45.0
        self.ultrasound_level = 0
        self.ultrasound_frequency = 0
        self.vibration_level = 0
        self.vibration_frequency = 0
        # Dashboard: last emitted AT#REPORT line and the device's last response.
        self.last_report_type = REPORT_TYPE_3300
        self.last_at_command = "(none yet)"
        self.last_response = "(waiting)"
        self.last_alarm_flags = ALARM_NONE
        self.last_priority = PRIO_NORMAL

    def drain_battery(self) -> None:
        with self.lock:
            if self.battery > 0:
                self.battery -= 1

    def regen_readings(self) -> None:
        with self.lock:
            self.temperature = round(random.uniform(-100.0, 100.0), 2)
            self.humidity = round(random.uniform(0.0, 100.0), 2)
            self.temperature2 = round(random.uniform(-100.0, 100.0), 2)
            self.humidity2 = round(random.uniform(0.0, 100.0), 2)
            self.ultrasound_level = random.randint(0, 100)
            self.ultrasound_frequency = random.randint(0, 255)
            self.vibration_level = random.randint(0, 500)
            self.vibration_frequency = random.randint(0, 255)

    def compute_alarm_flags(self) -> int:
        """Current alarm bitmask: battery-low, plus temp/humidity (both channels)
        and ultrasound/vibration out of the configured 3300 bounds (only checked
        once a config sets them)."""
        flags = ALARM_NONE
        with self.lock:
            if self.battery <= self.battery_level_min:
                flags |= ALARM_BATTERY
            if self.temp_max is not None and self.temp_min is not None:
                if self.temperature > self.temp_max or self.temperature < self.temp_min:
                    flags |= ALARM_TEMPERATURE1
            if self.hum_max is not None and self.hum_min is not None:
                if self.humidity > self.hum_max or self.humidity < self.hum_min:
                    flags |= ALARM_HUMIDITY1
            if self.temp_max2 is not None and self.temp_min2 is not None:
                if self.temperature2 > self.temp_max2 or self.temperature2 < self.temp_min2:
                    flags |= ALARM_TEMPERATURE2
            if self.hum_max2 is not None and self.hum_min2 is not None:
                if self.humidity2 > self.hum_max2 or self.humidity2 < self.hum_min2:
                    flags |= ALARM_HUMIDITY2
            if self.ultrasound_level_max is not None and self.ultrasound_level_max:
                if self.ultrasound_level > self.ultrasound_level_max:
                    flags |= ALARM_ULTRASOUND
            if self.vibration_level_max is not None and self.vibration_level_max:
                if self.vibration_level > self.vibration_level_max:
                    flags |= ALARM_VIBRATION
        return flags

    def snapshot_payload(self, alarm_flags: int) -> bytes:
        with self.lock:
            info = build_report_info_3300(
                temperature1=self.temperature, humidity1=self.humidity,
                temperature2=self.temperature2, humidity2=self.humidity2,
                ultrasound_level=self.ultrasound_level,
                ultrasound_frequency=self.ultrasound_frequency,
                vibration_level=self.vibration_level,
                vibration_frequency=self.vibration_frequency,
            )
            return build_report_payload(
                sn_hex=f"{self.sn:012X}", battery_level=self.battery,
                alarm_flags=alarm_flags, report_info=info,
                firmware_version=self.fw_version,
            )

    def apply_config(self, decoded: dict) -> list[str]:
        """Apply a decoded config; return human-readable notes on what changed."""
        notes: list[str] = []
        with self.lock:
            if "sleep_time_sec" in decoded and decoded["sleep_time_sec"]:
                self.sleep_time_sec = decoded["sleep_time_sec"]
                notes.append(f"sleep_time_sec -> {self.sleep_time_sec}s")
            if "battery_level_min" in decoded:
                self.battery_level_min = decoded["battery_level_min"]
                notes.append(f"battery_level_min -> {self.battery_level_min}%")
            cmd = decoded.get("command_raw", 0)
            if cmd & CMD_DEMO_MODE:
                self.demo_mode = True
                notes.append("DEMO_MODE on")
            if cmd & CMD_RESET:
                # RESET: restore battery to full, clear demo mode (best-effort sim).
                self.battery = 100
                self.demo_mode = False
                notes.append("RESET (battery=100, demo off)")
            if cmd == 0:
                notes.append("command NO (no-op)")
            if "config_flags_raw" in decoded:
                self.config_flags = decoded["config_flags_raw"]
            info = decoded.get("config_info_decoded") or decoded.get("config_info_3300")
            if info:
                self.temp_max = info.get("temperature_max1", self.temp_max)
                self.temp_min = info.get("temperature_min1", self.temp_min)
                self.hum_max = info.get("humidity_max1", self.hum_max)
                self.hum_min = info.get("humidity_min1", self.hum_min)
                self.temp_max2 = info.get("temperature_max2", self.temp_max2)
                self.temp_min2 = info.get("temperature_min2", self.temp_min2)
                self.hum_max2 = info.get("humidity_max2", self.hum_max2)
                self.hum_min2 = info.get("humidity_min2", self.hum_min2)
                self.ultrasound_level_max = info.get("ultrasound_level_max", self.ultrasound_level_max)
                self.ultrasound_center_frequency = info.get("ultrasound_center_frequency", self.ultrasound_center_frequency)
                self.vibration_level_max = info.get("vibration_level_max", self.vibration_level_max)
                notes.append(f"thresholds T1[{self.temp_min}..{self.temp_max}]°C "
                             f"H1[{self.hum_min}..{self.hum_max}]% "
                             f"T2[{self.temp_min2}..{self.temp_max2}]°C "
                             f"H2[{self.hum_min2}..{self.hum_max2}]% "
                             f"US<={self.ultrasound_level_max} "
                             f"VIB<={self.vibration_level_max}")
            self.last_config_ts = time.strftime("%H:%M:%S")
        return notes


# ---------------------------------------------------------------------------
# Serial driver
# ---------------------------------------------------------------------------

class SerialIO:
    def __init__(self, port: str, baud: int, timeout_s: float, rtscts: bool):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1, rtscts=rtscts)
        self.timeout_s = timeout_s
        self._wlock = threading.Lock()
        # Terminal AT responses (OK/ERROR/#...) surfaced by the reader thread so
        # flow-controlled senders (send_large_data) can pace on them.
        self.resp_q: "queue.Queue[str]" = queue.Queue(maxsize=128)
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def write(self, data: bytes) -> None:
        with self._wlock:
            self.ser.write(data)
            self.ser.flush()

    def request(self, line: str) -> list[str]:
        """Send one AT line and collect reply lines synchronously.

        Used only during startup provisioning, before the background config
        reader thread starts — so there's no contention on ser.read(). Reads
        until the line goes quiet (no guaranteed terminator) or timeout.
        """
        if not line.endswith("\r"):
            line += "\r"
        self.write(line.encode("ascii", errors="replace"))

        QUIET_GAP_S = 0.05
        deadline = time.monotonic() + self.timeout_s
        buf = bytearray()
        lines: list[str] = []
        got = False
        last_rx = None
        while True:
            chunk = self.ser.read(64)
            if chunk:
                got = True
                last_rx = time.monotonic()
                buf.extend(chunk)
                text = buf.decode("ascii", errors="replace")
                parts = text.replace("\r\n", "\n").replace("\r", "\n").split("\n")
                buf = bytearray(parts[-1], "ascii")
                for piece in parts[:-1]:
                    piece = piece.strip()
                    if piece:
                        lines.append(piece)
            else:
                now = time.monotonic()
                if got:
                    if last_rx is not None and (now - last_rx) >= QUIET_GAP_S:
                        tail = buf.decode("ascii", errors="replace").strip()
                        if tail:
                            lines.append(tail)
                        return lines
                elif now >= deadline:
                    return lines
                time.sleep(0.01)


# ---------------------------------------------------------------------------
# Startup SN provisioning
# ---------------------------------------------------------------------------

def _parse_sn_line(lines: list[str]) -> Optional[int]:
    """Extract the 64-bit SN from a '#SN: 0x...' reply line.

    The firmware (cmd_get_sn / cmd_set_sn) prints '#SN: 0x%08x%08x' — a 0x
    prefix followed by 16 hex digits. Returns None if no #SN line is present.
    """
    for ln in lines:
        u = ln.upper()
        if "#SN:" in u:
            after = u.split("#SN:", 1)[1].strip()
            if after.startswith("0X"):
                after = after[2:]
            hexpart = "".join(c for c in after if c in "0123456789ABCDEF")
            if hexpart:
                try:
                    return int(hexpart, 16)
                except ValueError:
                    return None
    return None


def provision_sn(io: SerialIO, want_sn: int) -> bool:
    """Ensure the device's serial number equals want_sn.

    1. Query with AT#SN?.
    2. If it already matches, continue.
    3. Otherwise set it with AT#SN="<16 hex>" and require an OK reply (and,
       if the device echoes #SN, that it now matches).

    Returns True to continue, False to abort the script.
    """
    want_hex = f"{want_sn:016X}"

    # 1) Read current SN.
    replies = io.request("AT#SN?")
    cur = _parse_sn_line(replies)
    is_err = any(r == "ERROR" for r in replies)

    if cur is not None:
        print(f"AT#SN? -> current SN = 0x{cur:016X}"
              + ("  (device reports it as unset/ERROR)" if is_err else ""))
    elif not replies:
        print("AT#SN? -> no response from device (check port/baud/flow-control)")
        return False
    else:
        print(f"AT#SN? -> could not parse SN from reply: {replies!r}")

    if cur == want_sn and not is_err:
        print(f"SN already matches 0x{want_hex}; continuing.")
        return True

    # 2) Set the desired SN.
    print(f"Setting SN to 0x{want_hex} via AT#SN=...")
    set_replies = io.request(f'AT#SN="{want_hex}"')
    if any(r == "ERROR" for r in set_replies):
        print(f"AT#SN= returned ERROR: {set_replies!r} — aborting.")
        return False
    if not any(r == "OK" for r in set_replies):
        # Some firmwares omit OK; fall back to verifying via #SN echo / re-query.
        echoed = _parse_sn_line(set_replies)
        if echoed != want_sn:
            print(f"AT#SN= did not confirm (no OK, reply={set_replies!r}) — aborting.")
            return False

    # 3) Verify by re-reading.
    verify = io.request("AT#SN?")
    now = _parse_sn_line(verify)
    if now == want_sn and not any(r == "ERROR" for r in verify):
        print(f"SN set and verified = 0x{now:016X}; continuing.")
        return True
    print(f"SN verification failed after set (read back {now}, reply={verify!r}) — aborting.")
    return False


# ---------------------------------------------------------------------------
# Dashboard rendering (overwrites in place each refresh)
# ---------------------------------------------------------------------------

# Set True once we've confirmed (or enabled) ANSI VT processing on this stdout.
_ANSI_OK = False

# Number of lines the previous frame printed, so the next frame can move the
# cursor up that many rows and overwrite in place (more portable than \033[2J).
_LAST_FRAME_LINES = 0


def _enable_ansi() -> bool:
    """Enable ANSI escape processing for stdout. On Windows 10+ this flips the
    console's VIRTUAL_TERMINAL_PROCESSING bit; elsewhere ANSI just works.

    Returns True if in-place redraw via escape codes should work."""
    if not sys.stdout.isatty():
        return False
    if sys.platform != "win32":
        return True
    try:
        import ctypes
        kernel32 = ctypes.windll.kernel32
        # -11 = STD_OUTPUT_HANDLE; 0x0004 = ENABLE_VIRTUAL_TERMINAL_PROCESSING
        h = kernel32.GetStdHandle(-11)
        mode = ctypes.c_uint32()
        if not kernel32.GetConsoleMode(h, ctypes.byref(mode)):
            return False
        kernel32.SetConsoleMode(h, mode.value | 0x0004)
        return True
    except Exception:
        return False


def _fmt(v, suffix="") -> str:
    return f"{v}{suffix}" if v is not None else "—"


def render_dashboard(state: SensorState) -> None:
    """Render the full status panel, overwriting the previous frame."""
    with state.lock:
        sn = state.sn
        rtype = state.last_report_type
        sleep_s = state.sleep_time_sec
        bat_min = state.battery_level_min
        t_max, t_min = state.temp_max, state.temp_min
        h_max, h_min = state.hum_max, state.hum_min
        t_max2, t_min2 = state.temp_max2, state.temp_min2
        h_max2, h_min2 = state.hum_max2, state.hum_min2
        us_max, vib_max = state.ultrasound_level_max, state.vibration_level_max
        cfg_flags = state.config_flags
        cfg_ts = state.last_config_ts
        battery = state.battery
        temp, hum = state.temperature, state.humidity
        temp2, hum2 = state.temperature2, state.humidity2
        us_lvl, us_freq = state.ultrasound_level, state.ultrasound_frequency
        vib_lvl, vib_freq = state.vibration_level, state.vibration_frequency
        alarm = state.last_alarm_flags
        at_cmd = state.last_at_command
        resp = state.last_response

    now_str = time.strftime("%Y-%m-%d %H:%M:%S")
    enc = "NOT_ENCRYPTED" if (cfg_flags & FLAG_NOT_ENCRYPTED) else "ENCRYPTED"

    lines = []
    lines.append(f"SN: {sn:012X} ({sn})   Type: 0x{rtype:04X}   Time: {now_str}")
    lines.append("-" * 100)
    lines.append("Current Config" + (f"   (updated {cfg_ts})" if cfg_ts else "   (defaults — no config yet)"))
    lines.append("-" * 100)
    lines.append(f"Report time: {sleep_s}s".ljust(36) + f"Min Battery Level: {bat_min}%")
    lines.append(f"Temp1 Max: {_fmt(t_max,'C')}".ljust(36) + f"Hum1 Max: {_fmt(h_max,'%')}")
    lines.append(f"Temp1 Min: {_fmt(t_min,'C')}".ljust(36) + f"Hum1 Min: {_fmt(h_min,'%')}")
    lines.append(f"Temp2 Max: {_fmt(t_max2,'C')}".ljust(36) + f"Hum2 Max: {_fmt(h_max2,'%')}")
    lines.append(f"Temp2 Min: {_fmt(t_min2,'C')}".ljust(36) + f"Hum2 Min: {_fmt(h_min2,'%')}")
    lines.append(f"Ultrasound level max: {_fmt(us_max)}".ljust(36) + f"Vibration level max: {_fmt(vib_max)}")
    lines.append(f"Config Flags: 0x{cfg_flags:02X} ({enc})")
    lines.append("-" * 100)
    lines.append("Report / Current Parameters")
    lines.append("-" * 100)
    lines.append(f"Current time: {time.strftime('%H:%M:%S')}".ljust(40) + f"Current Battery level: {battery}%")
    lines.append(f"Temp1: {temp}C".ljust(40) + f"Hum1: {hum}%")
    lines.append(f"Temp2: {temp2}C".ljust(40) + f"Hum2: {hum2}%")
    lines.append(f"Ultrasound: lvl={us_lvl} freq={us_freq}".ljust(40)
                 + f"Vibration: lvl={vib_lvl} freq={vib_freq}")
    lines.append(f"Alarm Flags: 0x{alarm:04X} ({alarm_text(alarm)})")
    lines.append("-" * 100)
    lines.append("AT Command")
    lines.append("-" * 100)
    lines.append(at_cmd)
    lines.append("-" * 100)
    lines.append("Response")
    lines.append("-" * 100)
    lines.append(resp)
    lines.append("-" * 100)

    global _LAST_FRAME_LINES
    if _ANSI_OK:
        # Reposition to the top of the previous frame and rewrite each line with
        # a trailing \033[K (erase-to-EOL) so leftover chars are cleared. We do
        # NOT print a trailing newline after the last line: that newline, when
        # the frame sits at the bottom row, scrolls the viewport and throws off
        # the next cursor-up count (which caused stacked header lines). The
        # cursor therefore rests on the final line; the next frame moves up
        # (lines-1) rows to land back on the first line. Avoids \033[2J/3J,
        # which some terminals ignore.
        out = []
        if _LAST_FRAME_LINES:
            out.append(f"\033[{_LAST_FRAME_LINES - 1}A")   # up to first line
        out.append("\r")
        out.append("\033[K\n".join(ln for ln in lines))    # lines joined, each cleared
        out.append("\033[K\033[J")                         # clear last line + below
        sys.stdout.write("".join(out))
        _LAST_FRAME_LINES = len(lines)
    else:
        sys.stdout.write("\n" + "\n".join(lines) + "\n")   # plain scroll fallback
    sys.stdout.flush()


# ---------------------------------------------------------------------------
# Config reader thread
# ---------------------------------------------------------------------------

def config_reader(io: SerialIO, state: SensorState, stop_evt: threading.Event) -> None:
    """Listen for AT#CONFIG pushes; CRC-verify, reply OK/ERROR, apply config,
    then refresh the dashboard."""
    buf = bytearray()
    while not stop_evt.is_set():
        try:
            chunk = io.ser.read(256)
        except Exception:
            break
        if not chunk:
            continue
        buf.extend(chunk)
        text = buf.decode("ascii", errors="replace")
        parts = text.replace("\r\n", "\n").replace("\r", "\n").split("\n")
        buf = bytearray(parts[-1], "ascii")
        for raw in parts[:-1]:
            line = raw.strip()
            if not line:
                continue
            # Surface terminal responses (OK/ERROR/#...) so flow-controlled
            # senders like send_large_data() can pace on them.
            if line in ("OK", "ERROR") or line.startswith("#"):
                try:
                    io.resp_q.put_nowait(line)
                except queue.Full:
                    pass
                continue
            if not line.startswith("AT#CONFIG="):
                continue
            ok, summary, parsed = parse_at_config(line)
            if not ok:
                io.write(b"\r\nERROR\r\n")
                with state.lock:
                    state.last_at_command = f"<< {line}"
                    state.last_response = f"AT#CONFIG rejected — {summary}"
                render_dashboard(state)
                continue
            io.write(f'\r\n#CONFIG: "{parsed["sn"]:016X}",'
                     f'"{parsed["data_id"]:04X}"\r\n'.encode("ascii"))
            io.write(b"\r\nOK\r\n")
            decoded = decode_config_payload(parsed["payload"])
            notes = state.apply_config(decoded) if decoded and "raw" not in decoded else []
            with state.lock:
                state.last_at_command = f"<< {line}"
                state.last_response = (f"AT#CONFIG accepted — {summary}"
                                       + (f"  | applied: {', '.join(notes)}" if notes else ""))
            render_dashboard(state)


# ---------------------------------------------------------------------------
# Main report loop
# ---------------------------------------------------------------------------

def make_dummy_sound_record(n: int) -> bytes:
    """Generate an n-byte dummy 'sound record' payload — a 0..255 ramp, matching
    the firmware's large-data test pattern (byte i = i & 0xFF). This is the
    sound_record inside data_info_3300, NOT the whole transfer, so it is capped
    so the wrapped sensor_large_data_structure_t blob still fits one transfer."""
    if n <= 0:
        return b""
    if n > LARGE_DATA_SOUND_RECORD_MAX:
        raise ValueError(f"sound_record {n} B exceeds max "
                         f"{LARGE_DATA_SOUND_RECORD_MAX} B")
    return bytes(i & 0xFF for i in range(n))


def ld_sound_record_len_for_total(total_size: int) -> int:
    """sound_record length so the WHOLE serialized sensor_large_data_structure_t
    (header + report_info_3300 + sound_record) is exactly total_size bytes. Lets
    test tools target a round whole-structure size — e.g. a "100 KB" transfer is
    100 KB total, header and report_info included, not 100 KB of sound on top."""
    sound_len = total_size - LARGE_DATA_HEADER_SIZE - REPORT_INFO_3300_SIZE
    if sound_len < 0:
        raise ValueError(f"total_size {total_size} B too small; minimum is "
                         f"{LARGE_DATA_HEADER_SIZE + REPORT_INFO_3300_SIZE} B "
                         f"(header + report_info_3300)")
    if total_size > SENSOR_LARGE_DATA_INFO_MAX:
        raise ValueError(f"total_size {total_size} B exceeds max "
                         f"{SENSOR_LARGE_DATA_INFO_MAX} B (200 KB whole structure)")
    return sound_len


def build_large_data_blob(state: "SensorState", sound_record: bytes,
                          data_type: int = LARGE_DATA_TYPE_3300) -> bytes:
    """Wrap raw sound_record samples into a serialized
    sensor_large_data_structure_t blob, taking the SN / firmware / battery and
    the current 3300 readings from `state` for the header and data_info_3300.
    This is what gets streamed over AT#LDINIT + AT#LD."""
    with state.lock:
        info = build_large_data_info_3300(
            sound_record,
            temperature1=state.temperature, humidity1=state.humidity,
            temperature2=state.temperature2, humidity2=state.humidity2,
            ultrasound_level=state.ultrasound_level,
            ultrasound_frequency=state.ultrasound_frequency,
            vibration_level=state.vibration_level,
            vibration_frequency=state.vibration_frequency,
        )
        return build_large_data_payload(
            sn_hex=f"{state.sn:012X}", firmware_version=state.fw_version,
            battery_level=state.battery, data_info=info, data_type=data_type,
        )


def _ld_blob_from_args(args, state: "SensorState") -> bytes:
    """Build the one-shot large-data blob — a serialized
    sensor_large_data_structure_t — from --send-ld FILE or --ld-bytes N. The
    file/dummy bytes become the sound_record inside data_info_3300; the struct
    header (name "ESC33", type 0x3300, SN, firmware, battery, ...) is added on
    top. Returns b'' if neither option was given."""
    if args.send_ld:
        with open(args.send_ld, "rb") as f:
            sound_record = f.read()
    elif args.ld_bytes > 0:
        sound_record = make_dummy_sound_record(args.ld_bytes)
    else:
        return b""

    if len(sound_record) > LARGE_DATA_SOUND_RECORD_MAX:
        raise ValueError(f"sound_record {len(sound_record)} B exceeds max "
                         f"{LARGE_DATA_SOUND_RECORD_MAX} B (the wrapped transfer "
                         f"must stay <= {LARGE_DATA_MAX_TRANSFER} B incl. header)")
    return build_large_data_blob(state, sound_record)


def _drain_resp_q(io: SerialIO) -> None:
    """Discard buffered AT responses so a stale OK/ERROR isn't mistaken for the
    ack of the next line we send."""
    try:
        while True:
            io.resp_q.get_nowait()
    except queue.Empty:
        pass


def _await_ack(io: SerialIO, timeout_s: float) -> Optional[str]:
    """Block until the device sends a TERMINAL response (OK/ERROR), discarding
    any intermediate '#...' info lines (e.g. the '#LDINIT:' echo cmd_ld_init
    emits before its OK). Returns the terminal token, or None on timeout.

    Waiting for the terminal — not merely the first line — is what keeps
    large-data strictly one-line-in-flight. If we returned on the '#LDINIT:'
    echo, its trailing 'OK' would be left in the queue and instantly consumed
    as the next chunk's ack, so the host would run a line ahead; two ~1 KB
    AT#LD lines then coalesce in the firmware's RX ring buffer, get split at
    the drain boundary, and wedge RX."""
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        try:
            tok = io.resp_q.get(timeout=remaining)
        except queue.Empty:
            return None
        if tok in ("OK", "ERROR"):
            return tok
        # An informational '#...' line — keep waiting for the terminal token.


def send_large_data(io: SerialIO, state: SensorState, blob: bytes,
                    data_id: int = 1, ack_timeout_s: float = LD_ACK_TIMEOUT_S) -> None:
    """Stream a binary blob to the sensor board: one AT#LDINIT announcing the
    transfer, then a sequence of AT#LD serial chunks (<= AT_LD_PAYLOAD_MAX bytes
    each), paged/chunked and per-chunk CRC16'd.

    FLOW-CONTROLLED: waits for the device's response (OK/ERROR) after each line
    before sending the next, so there is never more than one ~1 KB line in
    flight. This is required — the firmware's AT RX buffer cannot absorb
    back-to-back lines (it disables RX and wedges). If no response arrives
    within ack_timeout_s we proceed anyway (degraded pacing)."""
    total = len(blob)
    if total == 0:
        return
    chunks = list(iter_ld_chunks(blob))
    n = len(chunks)
    last_chunk_size = len(chunks[-1][2])
    data_crc32 = zlib.crc32(blob) & 0xFFFFFFFF
    _drain_resp_q(io)

    # 1) Announce the transfer, then wait for the device to consume it.
    init_line = build_at_ldinit(state.sn, data_id, total, n, last_chunk_size,
                                data_crc32)
    io.write(init_line.encode("ascii", errors="replace"))
    with state.lock:
        state.last_at_command = (f">> AT#LDINIT id={data_id} total={total} "
                                 f"chunks={n}")
        state.last_response = f"large-data: init sent ({total} B, {n} chunks)"
    render_dashboard(state)
    _await_ack(io, ack_timeout_s)

    # 2) Stream the chunks, one in flight at a time (wait for ack between each).
    for i, (page, chunk, payload) in enumerate(chunks):
        line = build_at_ld(state.sn, data_id, page, chunk, payload)
        io.write(line.encode("ascii", errors="replace"))
        ack = _await_ack(io, ack_timeout_s)
        with state.lock:
            state.last_at_command = (f">> AT#LD p{page:02X} c{chunk:02X} "
                                     f"({len(payload)} B)")
            state.last_response = (f"large-data: chunk {i + 1}/{n} "
                                   f"[{ack or 'timeout'}]")
        render_dashboard(state)

    with state.lock:
        state.last_response = f"large-data: done ({total} B in {n} chunks)"
    render_dashboard(state)


def run(io: SerialIO, state: SensorState, args, stop_evt: threading.Event) -> None:
    # data_id auto-increments per report; first send uses args.data_id.
    data_id = max(1, min(DATA_ID_MAX, args.data_id))

    # One-shot large-data transfer on startup, if requested.
    ld_blob = _ld_blob_from_args(args, state)
    if ld_blob:
        send_large_data(io, state, ld_blob, data_id=args.ld_data_id)

    now = time.monotonic()
    next_battery = now + args.battery_period
    next_check   = now + args.gen_period   # parameter-check (and regen) cadence
    next_report  = now                     # send one promptly on startup

    def send(reason: str, alarm_flags: int) -> None:
        nonlocal data_id
        # Priority rule: low priority (2) when no alarm, high (0) on an alarm.
        priority = PRIO_ALARM if alarm_flags else PRIO_NORMAL
        payload = state.snapshot_payload(alarm_flags)
        line = build_at_report(state.sn, data_id, payload, priority)
        io.write(line.encode("ascii", errors="replace"))
        with state.lock:
            state.last_alarm_flags = alarm_flags
            state.last_priority = priority
            state.last_at_command = f">> {line.strip()}"
            state.last_response = f"sent ({reason}, id={data_id}, prio={priority})"
        render_dashboard(state)
        data_id = next_data_id(data_id)        # advance for the next report

    while not stop_evt.is_set():
        now = time.monotonic()
        dirty = False

        # 1) Battery drain.
        if now >= next_battery:
            state.drain_battery()
            next_battery += args.battery_period
            dirty = True

        # 2) Every gen_period: regenerate readings, then CHECK PARAMETERS.
        #    If any alarm condition holds, send a report immediately (high prio).
        #    Otherwise the readings stay buffered for the next interval report.
        if now >= next_check:
            state.regen_readings()
            next_check += args.gen_period
            dirty = True
            alarm = state.compute_alarm_flags()
            if alarm:
                send("alarm-check", alarm)
                # Re-anchor the interval so we don't double-send right after.
                next_report = now + state.sleep_time_sec
                dirty = False

        # 3) Scheduled interval report (normal periodic, whatever the alarm).
        if now >= next_report:
            send("interval", state.compute_alarm_flags())
            next_report = now + state.sleep_time_sec
            dirty = False

        # Refresh the panel for clock/battery/reading changes between sends.
        if dirty:
            with state.lock:
                state.last_alarm_flags = state.compute_alarm_flags()
            render_dashboard(state)

        stop_evt.wait(0.5)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def autodetect_port() -> Optional[str]:
    cands = list(list_ports.comports())
    if not cands:
        return None
    pref = [p for p in cands if any(k in (p.description or "").lower()
                                    for k in ("nrf", "jlink", "j-link", "segger"))]
    return (pref[0] if pref else cands[0]).device


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port", help="serial port (auto-detect if omitted)")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    p.add_argument("--no-rtscts", action="store_true",
                   help="disable RTS/CTS (firmware default has it ON)")
    p.add_argument("--list", action="store_true", help="list serial ports and exit")

    p.add_argument("--sn", required=False, help="sensor serial number, hex")
    p.add_argument("--data-id", type=lambda s: int(s, 0), default=1)
    p.add_argument("--firmware-version", type=lambda s: int(s, 0), default=1)

    p.add_argument("--battery", type=int, default=100, help="start battery %% (default 100)")
    p.add_argument("--battery-min", type=int, default=DEFAULT_BATTERY_MIN,
                   help=f"battery_level_min alarm threshold before any config "
                        f"(default {DEFAULT_BATTERY_MIN})")
    p.add_argument("--default-interval", type=float, default=DEFAULT_INTERVAL_S,
                   help=f"sleep_time_sec until a config sets it "
                        f"(default {DEFAULT_INTERVAL_S})")

    # Timer overrides (real defaults: 300 / 120). Lower them to test quickly.
    p.add_argument("--battery-period", type=float, default=BATTERY_PERIOD_S,
                   help="seconds per 1%% battery drain (default 300)")
    p.add_argument("--gen-period", type=float, default=GEN_PERIOD_S,
                   help="seconds between reading regeneration (default 120)")

    p.add_argument("--send-ld", metavar="FILE",
                   help="send FILE's bytes as the sound_record of a one-shot "
                        "AT#LD large-data transfer at startup (wrapped in a "
                        "sensor_large_data_structure_t)")
    p.add_argument("--ld-bytes", type=lambda s: int(s, 0), default=0,
                   help="use N generated dummy bytes as the sound_record of a "
                        "one-shot AT#LD transfer (0 = off; mutually exclusive "
                        "with --send-ld)")
    p.add_argument("--ld-data-id", type=lambda s: int(s, 0), default=1,
                   help="data_id for the AT#LDINIT/AT#LD large-data transfer (default 1)")
    p.add_argument("--packed", action="store_true",
                   help="deprecated/no-op: the wire layout is now fixed (packed, "
                        "fixed-width) and no longer compiler-dependent")
    p.add_argument("--skip-sn-check", action="store_true",
                   help="skip the startup AT#SN?/AT#SN= provisioning handshake")
    p.add_argument("--no-clear", action="store_true",
                   help="don't clear the screen each refresh; scroll panels instead")

    args = p.parse_args(argv)

    global _ANSI_OK
    _ANSI_OK = (not args.no_clear) and _enable_ansi()

    if args.list:
        for d in list_ports.comports():
            print(f"{d.device}\t{d.description}")
        return 0
    if not args.sn:
        sys.stderr.write("error: --sn is required (unless using --list)\n")
        return 2
    try:
        sn = int(args.sn, 16)
    except ValueError:
        sys.stderr.write(f"error: --sn not valid hex: {args.sn!r}\n")
        return 2
    if not (0 <= sn < (1 << 64)):
        sys.stderr.write("error: --sn out of range for uint64\n")
        return 2

    port = args.port or autodetect_port()
    if not port:
        sys.stderr.write("error: no serial port given and none auto-detected.\n")
        return 2
    try:
        io = SerialIO(port, args.baud, args.timeout, rtscts=not args.no_rtscts)
    except serial.SerialException as e:
        sys.stderr.write(f"error: cannot open {port}: {e}\n")
        return 2

    # Provision the device SN before doing anything else, so the SN we put in
    # AT#REPORT lines actually matches the device. Runs synchronously before
    # the background config reader starts (no contention on ser.read()).
    if not args.skip_sn_check:
        if not provision_sn(io, sn):
            io.close()
            return 1

    state = SensorState(
        sn=sn, battery=max(0, min(100, args.battery)),
        fw_version=args.firmware_version,
        sleep_time_sec=int(args.default_interval),
        battery_level_min=args.battery_min,
    )

    render_dashboard(state)         # initial frame before any send/config

    stop_evt = threading.Event()
    reader = threading.Thread(target=config_reader, args=(io, state, stop_evt),
                              name="config-reader", daemon=True)
    reader.start()

    try:
        run(io, state, args, stop_evt)
    except KeyboardInterrupt:
        print("\n(exit)")
    finally:
        stop_evt.set()
        reader.join(timeout=0.3)
        io.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
