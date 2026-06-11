# Project Overview (for AI agents)

> Read this first. It orients you on what this project is, how it's built, where things live, and
> the conventions/pitfalls to respect before making any change. For a fuller human-facing
> description see [`README.md`](./README.md).

## TL;DR

Embedded C firmware for a **DECT NR+ multi-hop mesh network** on Nordic **nRF9151** SiPs, built on
**nRF Connect SDK / Zephyr RTOS (NCS v3.2.4)** using the `nrf_modem_dect_phy` modem interface. One
codebase produces three roles — **Gateway**, **Anchor**, **Sensor** — selected at boot by GPIO
straps. The mesh auto-pairs, discovers routes, and moves reports / config / large data across hops
with per-packet ACK + retry.

## Environment

- **OS / shell:** Windows + PowerShell. Use PowerShell syntax (`$env:VAR`, `$null`, backtick line
  continuation), not bash.
- **SDK:** NCS **v3.2.4** under `c:/ncs/` (`ZEPHYR_BASE = c:/ncs/v3.2.4/zephyr`, toolchain
  `c:/ncs/toolchains/fd21892d0f`). Pinned by `build.ps1`.
- **Board:** `nrf9151dk/nrf9151/ns` (non-secure). Also builds for `nrf9161dk/nrf9161/ns`.
- **Modem firmware:** DECT NR+ PHY firmware `mfw-nr+-phy_nrf91x1_2.0.0` (not public; Nordic sales).

## Build / flash / iterate

```powershell
./setup_env.ps1          # one-time: venv + west + NCS python deps
./build.ps1              # full build first run, incremental after; output -> build/merged.hex
./program.ps1 <1..6|all> # flash a mapped lab board by index
./program.ps1 1 -recover # recover + modem-fw upgrade + app flash (fresh board)
```

- Don't hand-roll `west build` flags — `build.ps1` sets the toolchain/board/sysbuild correctly.
- `build/` is git-ignored. A generated devicetree exists at `build/hello_dect/zephyr/zephyr.dts`
  if you need to confirm node labels/aliases.
- There is **no host unit-test suite**; verification is on-target. `sample.yaml` is Twister CI
  metadata only.

## Architecture in one screen

`main.c` brings up modem → DECT PHY (init/configure/activate) → `product_info_init()` (reads role
straps) → `storage_init()` (EEPROM) → `psram_init()` → `slm_at_init()` → `buttons_init()` /
`led_init()`, then runs a **cooperative super-loop**. Each subsystem exposes a `*_tick()` /
`*_run()` called once per iteration — **nothing blocks**; long work is sliced across ticks and
timeouts are non-blocking (`timeout.c`).

```
main_sub_run()     RX/TX state machine, per-role RX windows
slm_at_run_cycle() AT command parse/dispatch (UART1)
mesh_tick()        pairing / response-window timers
tracker_tick()     retry expired packets
config_tick() / report_tick() / large_data_tick()   transport lifecycles
known_devices_tick()  ping/keepalive
```

### Where things live (`src/`)

| Concern | Files |
|---|---|
| Entry + loop / RX-TX SM | `main.c`, `main_sub.c` |
| DECT PHY wrapper | `radio.c` |
| Mesh core + storage helpers | `mesh.c` |
| Pairing / routing / session | `mesh_layers/mesh_pairing.c`, `mesh_routing.c`, `mesh_session.c` |
| Identity / role / hop# / known devices | `product_info.c` |
| Transport: reports / config / large | `data.c`, `config.c`, `large_data.c` |
| Reliability primitives | `tracker.c`, `timeout.c`, `queue.c` |
| Persistence | `storage.c` (EEPROM), `psram.c`, `spi_bus.c` |
| Host AT server | `slm_at_main.c`, `slm_at_sub.c`, `slm_at_uart.c` |
| **Test hooks (DK only)** | `testing/buttons.c`, `testing/led.c` |
| Wire protocol (read this for packet layout) | `protocol.h` |

Host-side Python tools (AT console, GUIs, sensor sim) live in `tools/`.

## Domain facts an agent must know

- **Roles are GPIO-strapped** at boot via active-low pins **P0.21 / P0.23** (decoded in
  `product_info_init()`): `00`=Gateway, `01`=Anchor, `10`/`11`=Sensor. Code is identical across
  roles; behaviour branches on `get_device_type()`.
- **`hop_num` = hop count from gateway.** Gateway = `0`; unpaired/disconnected = **`0xFF`** (no
  route). This sentinel is used widely (e.g., the LED1 status blink). Don't repurpose `0xFF`.
- **Every packet** starts with the 7-byte `packet_header_t` and carries a `tracking_id` (1–254)
  used for ACK matching and retries. Sending a packet generally means registering it with
  `tracker_*` so it retries on timeout (`PACKET_MAX_RETRIES`, `PACKET_TIMEOUT_MS`).
- **Three transport planes**, each with `*_INIT` / `*_CHUNK` / `*_RECEIVED` request+ACK pairs:
  reports (≤256 B), config (≤128 B), large data (≤200 KB, paged via PSRAM). `SEND_DATA_MAX = 180 B`
  per chunk (sized to DECT subslots @ MCS 2).
- **Persistence split:** Gateway holds the full mesh table in EEPROM (12 KB partition); anchors
  hold local infra+sensor tables. PSRAM (8 MB) is bulk staging only — see the partition maps in
  `psram.h` and `storage.h`. Both have compile-time `_Static_assert` capacity checks.
- **Sizing constants** (`protocol.h`): `MAX_DEPTH=4`, `MAX_ANCHORS=8`, `MAX_SENSORS=64`,
  `MAX_DEVICES=256`. Changing these can overflow EEPROM partitions (the asserts will catch it).

## Conventions to match

- **C / Zephyr style:** tabs for indentation; `snake_case`; one `LOG_MODULE_REGISTER` per module
  with a `CONFIG_<MODULE>_LOG_LEVEL` knob (declared in `Kconfig`, set in `prj.conf`).
- **Wire structs** are `__attribute__((packed))` with a trailing `crc32`/`config_crc16` field that
  must stay last (CRC covers everything before it). Don't reorder fields casually.
- **GPIO/peripherals** come from devicetree: `DT_NODELABEL(...)` / `DT_ALIAS(...)` +
  `GPIO_DT_SPEC_GET`. New board wiring goes in `nrf9151dk_nrf9151_ns.overlay`.
- **New source files** must be added to `CMakeLists.txt` (`target_sources(app PRIVATE ...)`).
  Test-only code is grouped under the "Testing utilities" section.
- **Don't block** in tick functions or handlers; keep per-tick work bounded (see the
  `PROCESS_*_SLOTS` caps). Button/LED handlers run in a thread context (not ISR) and may log.

## Gotchas

- `CONFIG_CARRIER` **must** be a region-legal value; a `BUILD_ASSERT` in `main.c` fails the build
  if it's `0`. Default here is `525` (EU band 1).
- This is a **non-secure (`ns`) build** with sysbuild (TF-M + MCUboot child images). Clearing
  `CONF_FILE`/`BOARD_ROOT` (as `build.ps1` does) matters so they don't leak into child builds.
- The original Nordic README and `sample.yaml` mention `overlay-eu.conf` / `overlay-us.conf`, but
  **those files are not in this repo** — configuration is via `prj.conf` + `Kconfig` instead.
- `OTA_*` packet types exist in `protocol.h` but OTA is **not implemented** (`ota_tick()` is
  commented out in `main.c`). Treat as planned.

## Before you finish

- If you add a tick/handler, wire it into the `main.c` loop and `CMakeLists.txt`.
- If you touch wire structs or sizing constants, re-check the `_Static_assert`s and that both
  sender and receiver agree.
- Build with `./build.ps1` to confirm it compiles before claiming done; on-target behaviour can't
  be unit-tested here.
