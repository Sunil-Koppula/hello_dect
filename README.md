# DECT NR+ PHY Mesh

A self-organizing wireless **mesh network** built on the **DECT NR+** physical layer, running on
Nordic Semiconductor **nRF9151** SiPs. The firmware turns a set of nRF9151 DK boards into a
multi-hop network of **Gateway**, **Anchor**, and **Sensor** devices that pair automatically,
discover routes, and move telemetry (reports), configuration, and large file transfers reliably
across hops.

> Built on the [nRF Connect SDK](https://github.com/nrfconnect/sdk-nrf) **v3.2.4** and the
> `nrf_modem_dect_phy` interface of the Modem library. It evolved from Nordic's
> [`dect_phy/hello_dect`](https://github.com/nrfconnect/sdk-nrf/blob/v3.2.4/samples/dect/dect_phy/hello_dect/README.rst)
> sample into a full application-layer mesh stack.

---

## Table of contents

- [What it does](#what-it-does)
- [Hardware](#hardware)
- [Device roles](#device-roles)
- [Architecture](#architecture)
- [Wire protocol](#wire-protocol)
- [Memory & storage layout](#memory--storage-layout)
- [Repository layout](#repository-layout)
- [Prerequisites](#prerequisites)
- [Building](#building)
- [Flashing](#flashing)
- [Configuration](#configuration)
- [Host tools](#host-tools)
- [Testing helpers](#testing-helpers)
- [Regulatory notice](#regulatory-notice)

---

## What it does

- **Automatic pairing** — devices discover and pair with the nearest upstream neighbour using a
  challenge/response handshake (`PAIR_REQUEST` → `PAIR_RESPONSE` → `PAIR_CONFIRM` → `PAIR_ACK`).
- **Multi-hop routing** — each device tracks its **hop count from the gateway**; routes are
  established on demand via route-discovery packets (up to `MAX_DEPTH = 4` hops).
- **Reliable transport** — every packet is tracked, acknowledged, and retried (`PACKET_MAX_RETRIES`)
  with per-packet timeouts and a tracking-ID scheme.
- **Three data planes**:
  - **Reports** — small sensor telemetry (≤ 256 B), chunked and reassembled.
  - **Config** — gateway-pushed device configuration (≤ 128 B) with CRC validation.
  - **Large data** — bulk transfers up to 200 KB, paged and chunked through external PSRAM.
- **Network self-healing** — periodic pinging of known devices, communication-failure counting,
  and re-pairing when a parent is lost.
- **Persistent topology** — the gateway keeps the full mesh table in EEPROM; anchors keep their
  local infra/sensor tables.
- **Host interface** — an SLM-style **AT command** transport over UART1 for injecting/extracting
  data from a PC (see [Host tools](#host-tools)).

---

## Hardware

| Item | Detail |
|------|--------|
| Board | **nRF9151 DK** (`nrf9151dk/nrf9151/ns`) — also builds for `nrf9161dk/nrf9161/ns` |
| Modem firmware | **DECT NR+ PHY firmware** `mfw-nr+-phy_nrf91x1_2.0.0` (Nordic, contact sales) |
| External flash | GD25WB256 (SPI3, `nordic,pm-ext-flash`) |
| External PSRAM | APS6404L — 8 MB (SPI3, CS = P0.10) |
| EEPROM | M24M02 256 KB (I²C2: SDA = P0.30, SCL = P0.31) — topology storage |
| AT host UART | UART1 @ 1 Mbaud, HW flow control (TX P0.29 / RX P0.28 / CTS P0.17 / RTS P0.16) |
| Console / logs | UART0 (board default) |

### Device-type straps

The role is selected at boot by two **active-low, pulled-up** GPIO straps (`P0.21`, `P0.23`):

| P0.21 | P0.23 | Role |
|:----:|:----:|------|
| 0 | 0 | **Gateway** |
| 0 | 1 | **Anchor** |
| 1 | 0 | **Sensor** |
| 1 | 1 | Sensor (reserved → defaults to Sensor) |

`0` = pin grounded, `1` = floating (pull-up). Decoded in `product_info_init()`.

---

## Device roles

| Role | Hop # | Responsibilities |
|------|:-----:|------------------|
| **Gateway** | `0` | Root of the tree. Holds the full mesh table, pushes config, sinks all reports/large data, bridges to a PC over AT. |
| **Anchor** | `1…N` | Relays. Forward traffic up/down, store local infra + sensor tables, extend coverage. |
| **Sensor** | `0xFF` until paired | Leaf nodes. Generate reports and large data; sleep otherwise. |

A device whose `hop_num == 0xFF` has **no route to the gateway** (unpaired / disconnected). This is
the signal the status LED uses (see [Testing helpers](#testing-helpers)).

---

## Architecture

The application runs a cooperative super-loop in `main.c`. After bringing up the modem, DECT PHY,
storage, and PSRAM, it ticks each subsystem in turn:

```
main()
 ├─ nrf_modem_lib_init / dect_phy init → configure → activate
 ├─ product_info_init()      # read straps → role, IDs, serial, hop#
 ├─ storage_init()           # EEPROM partitions
 ├─ psram_init()             # 8 MB external RAM
 ├─ slm_at_init()            # AT UART transport
 ├─ buttons_init() / led_init()   # testing helpers
 └─ while (1):
      main_sub_run()         # RX/TX state machine (per-role RX windows)
      slm_at_run_cycle()     # parse + dispatch AT commands
      mesh_tick()            # pairing/response-window timers
      tracker_tick()         # retry expired packets
      config_tick()          # config slot lifecycle
      report_tick()          # report slot lifecycle
      large_data_tick()      # large-transfer lifecycle
      known_devices_tick()   # ping/keepalive known neighbours
```

### Module map

| Layer | Files | Purpose |
|-------|-------|---------|
| **Radio / PHY** | `radio.c/.h`, `main_sub.c/.h` | `nrf_modem_dect_phy` wrapper, TX/RX, per-role RX-window state machine |
| **Mesh core** | `mesh.c/.h` | Mesh init/tick, network time, infra/sensor/mesh storage helpers |
| **Mesh layers** | `mesh_layers/mesh_pairing.c`, `mesh_routing.c`, `mesh_session.c` | Pairing handshake, route discovery/info, RX dispatch + session handling |
| **Identity** | `product_info.c/.h` | Role/ID/serial/hop#, known-device tracking, factory reset |
| **Transport — reports** | `data.c/.h` | Small report sender + reassembly slots |
| **Transport — config** | `config.c/.h` | Gateway→device config push, validation, retries |
| **Transport — large** | `large_data.c/.h` | Paged/chunked bulk transfer (≤ 200 KB) via PSRAM |
| **Reliability** | `tracker.c/.h`, `timeout.c/.h`, `queue.c/.h` | Per-packet retry tracking, non-blocking timeouts, TX/RX queues |
| **Persistence** | `storage.c/.h` (EEPROM), `psram.c/.h` (PSRAM), `spi_bus.c` | Topology tables + bulk staging |
| **Host AT** | `slm_at_main.c`, `slm_at_sub.c`, `slm_at_uart.c` | SLM-style AT command server over UART1 |
| **Testing** | `testing/buttons.c`, `testing/led.c` | DK buttons (inject traffic / factory reset) + status LED |

---

## Wire protocol

Defined in `src/protocol.h`. Every packet begins with a common 7-byte header:

```c
typedef struct {
    uint8_t  packet_type;   /* packet_type_t   */
    uint8_t  device_type;   /* device_type_t   */
    uint8_t  priority;      /* HIGH / MEDIUM / LOW */
    uint8_t  tracking_id;   /* 1–254, for ACK matching + retries */
    uint16_t device_id;     /* sender or target short ID */
    uint8_t  status;        /* STATUS_* code   */
} __attribute__((packed)) packet_header_t;
```

Packet families (each with request/ACK pairs):

- **Pairing** — `PAIR_*`, `REPAIR_*`, `JOINED_NETWORK*`
- **Liveness** — `PING_DEVICE`, `PING_ACK`, `DEVICE_UPDATED*`
- **Routing** — `ROUTE_DISCOVERY*`, `ROUTE_INFO*`
- **Reports** — `REPORT_INIT*`, `REPORT_CHUNK*`, `REPORT_RECEIVED*`
- **Config** — `CONFIG*`, `CONFIG_RECEIVED*`
- **Large data** — `LARGE_DATA_INIT*`, `LARGE_DATA_CHUNK*`, `LARGE_DATA_RECEIVED*`
- **OTA** — `OTA_*` (reserved / planned)

Key sizing constants: `MAX_DEPTH = 4`, `MAX_ANCHORS = 8`, `MAX_SENSORS = 64`,
`MAX_DEVICES = 256`, `SEND_DATA_MAX = 180 B` per chunk (fits within DECT subslots at MCS 2).

---

## Memory & storage layout

### EEPROM (M24M02, 256 KB) — persistent topology

Each partition starts with a 4-byte header (`magic 0xDE 0xC7` + `uint16` entry count).

| Partition | Offset | Size | Owner | Contents |
|-----------|:------:|:----:|-------|----------|
| Infra | `0x00000` | 128 B | Gateway + Anchor | Connected gateway/anchors (`infra_entry_t`) |
| Sensors | `0x00080` | 1 KB | Gateway + Anchor | Connected sensors (`sensor_entry_t`) |
| Mesh table | `0x00480` | 12 KB | Gateway only | Full topology (`mesh_entry_t`) |

Partition capacity is checked at compile time via `_Static_assert`.

### PSRAM (APS6404L, 8 MB) — bulk staging

| Region | Base | Size | Use |
|--------|:----:|:----:|-----|
| Tracker | `0x000000` | 32 KB | Retry payloads |
| Config | `0x008000` | 80 KB | Config slot staging |
| Data | `0x100000` | 1 MB | Small report slots (≤ 256 B) |
| Large data | `0x200000` | 6 MB | Large-transfer slots (20-chunk pages, 3,600 B each) |

---

## Repository layout

```
hello_dect/
├─ CMakeLists.txt              # app sources (+ testing utilities)
├─ Kconfig                     # CARRIER, MCS, TX_POWER, per-module log levels
├─ prj.conf                    # Zephyr/NCS config (modem, GPIO, I2C, SPI, UART async…)
├─ nrf9151dk_nrf9151_ns.overlay# devicetree: straps, PSRAM, EEPROM, UART1
├─ sample.yaml                 # Twister CI metadata
├─ build.ps1 / program.ps1     # Windows build + multi-board flash helpers
├─ setup_env.ps1               # Python venv + west/NCS deps
├─ src/
│  ├─ main.c, main_sub.c       # entry point + RX/TX state machine
│  ├─ radio.c                  # DECT PHY wrapper
│  ├─ product_info.c           # role/identity, known-device tracking
│  ├─ mesh.c, mesh_layers/     # mesh core + pairing/routing/session
│  ├─ data.c, config.c, large_data.c   # three transport planes
│  ├─ tracker.c, timeout.c, queue.c    # reliability primitives
│  ├─ storage.c, psram.c, spi_bus.c    # persistence
│  ├─ slm_at_*.c               # AT command server
│  └─ testing/buttons.c, led.c # DK test hooks
└─ tools/                      # Python host GUIs / simulators
```

---

## Prerequisites

- **nRF Connect SDK v3.2.4** with the matching toolchain (`v3.2.4`). The helper scripts assume a
  Windows install under `c:/ncs/` (`c:/ncs/v3.2.4`, `c:/ncs/toolchains/fd21892d0f`).
- **west**, Python 3, and the NCS Python requirements (`setup_env.ps1` creates the venv).
- **DECT NR+ PHY modem firmware** `mfw-nr+-phy_nrf91x1_2.0.0.zip` (place at `c:\ncs\`).
- `nrfjprog` / `nrfutil` for flashing (Nordic Command Line Tools).

> **Note:** the DECT NR+ PHY firmware is not publicly distributed — contact Nordic Semiconductor
> sales for availability.

---

## Building

First-time environment setup (creates the venv and installs west + NCS deps):

```powershell
./setup_env.ps1
```

Build (full build first time, incremental thereafter):

```powershell
./build.ps1
```

`build.ps1` activates the venv, pins `NCS_TOOLCHAIN_VERSION=v3.2.4` / `ZEPHYR_BASE`, then runs:

```
west build -d build --board nrf9151dk/nrf9151/ns --sysbuild .
```

This is a **non-secure (`ns`) sysbuild**: the app is the non-secure image, and sysbuild also
builds the **TF-M** secure firmware and the **MCUboot** bootloader as child images. The three are
combined into the merged image at `build/merged.hex`.

---

## Flashing

`program.ps1` flashes one or all of the lab boards by index (serial numbers are mapped inside the
script):

```powershell
./program.ps1 1            # flash app to board 1
./program.ps1 all          # flash app to every mapped board
./program.ps1 1 -recover   # full recover + modem-firmware upgrade + app flash
```

Use `-recover` for a fresh board: it recovers the device, programs the DECT NR+ modem firmware,
then flashes the app. **Set the device-type straps** (P0.21 / P0.23) before boot to pick the role.

---

## Configuration

Main knobs live in `Kconfig` / `prj.conf`:

| Option | Default | Meaning |
|--------|:-------:|---------|
| `CONFIG_CARRIER` | `525` | DECT carrier (band-1 range 525–551; see ETSI TS 103 636-2 §5.4.2). |
| `CONFIG_MCS` | `2` | Modulation & coding scheme — bytes per subslot. |
| `CONFIG_TX_POWER` | `13` | TX power (0–13; HW-limited). See ETSI TS 103 636-4 table 6.2.1-3. |
| `CONFIG_NETWORK_ID` | `91` | Mesh network ID. |
| `CONFIG_*_LOG_LEVEL` | `3` (prj.conf) | Per-module log verbosity (main, radio, mesh, data, …). |

> ⚠️ **`CONFIG_CARRIER` must be set to a value legal in your region.** A `BUILD_ASSERT` in
> `main.c` fails the build if it is left at `0`.

---

## Host tools

Python utilities under `tools/` talk to a Gateway over the AT UART (UART1):

| Tool | Purpose |
|------|---------|
| `at_console.py` | Interactive AT command console. |
| `gateway_gui.py` | GUI for monitoring the gateway / mesh. |
| `sensor_gui.py` | GUI representing a sensor endpoint. |
| `sensor_sim.py` | Sensor traffic simulator. |
| `sensor_structure.c` / `config.bin` | Shared application-layer payload schema (report/config/large-data structures for sensor types `0x3100`–`0x3300`), mirrored by the Python tools. |

---

## Testing helpers

`src/testing/` contains DK-only hooks (compiled in via `CMakeLists.txt`):

### Buttons (`buttons.c`)

Press-on-release handlers wired in `buttons_init()`:

| Button | Action |
|:------:|--------|
| **1** | Factory reset (erase storage) + cold reboot. |
| **2** | Sensor: build & send a dummy 256 B report. |
| **3** | Gateway: push a test config to a known device. Sensor: send a 100 KB large-data transfer. |
| **4** | Unassigned. |

### Status LED (`led.c`)

**LED1** mirrors the device's own hop number:

- `get_hop_number() == 0xFF` (no route to gateway) → **LED1 blinks** (250 ms toggle).
- any other hop number (connected) → **LED1 OFF**.

A low-priority thread polls the hop number and drives `DT_ALIAS(led1)` accordingly.

---

## Regulatory notice

DECT NR+ channel availability and the regulations governing them vary by country. Configure
`CONFIG_CARRIER`, `CONFIG_TX_POWER`, and `CONFIG_MCS` according to local rules
(see ETSI TS 103 636 series). Operating outside the permitted band may interfere with other radio
devices, including LTE.

---

*Copyright © 2023–2025 IRISS Inc. / Nordic Semiconductor ASA. Based on the nRF Connect SDK
`hello_dect` sample.*
