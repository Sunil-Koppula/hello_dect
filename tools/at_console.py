#!/usr/bin/env python3
"""AT command client for the hello_dect firmware (Gateway / Anchor / Sensor).

The firmware exposes an AT-style line protocol on uart1 (see src/serial.c).
The wire format echoes 3GPP modems:

    host -> device:   AT<suffix><CR>
    device -> host:   <CR><LF>+TAG: value<CR><LF>     (optional data line)
                      <CR><LF>OK<CR><LF>              (terminator)
                      <CR><LF>ERROR<CR><LF>           (on failure)

Echo is enabled by default; this script issues ATE0 at startup so the
response stream is unambiguous.

Examples:
    python at_console.py --port COM7                       # interactive REPL
    python at_console.py --port COM7 AT#DEVTYPE?           # one-shot
    python at_console.py --port /dev/ttyACM0 AT#VERSION? AT#HOP?

    # Push a config payload to a device, addressed by its SN:
    python at_console.py --port COM26 \
        --send-config 0x004AD000DEADBEEF --file config.bin

Requires: pyserial  (pip install pyserial)
"""

from __future__ import annotations

import argparse
import sys
import time
import zlib
from pathlib import Path
from typing import Iterable

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.stderr.write("error: pyserial not installed. Run: pip install pyserial\n")
    sys.exit(1)


DEFAULT_BAUD = 1000000  # uart1 current-speed in nrf9151dk_nrf9151_ns.overlay
DEFAULT_TIMEOUT_S = 2.0

# Must match firmware protocol.h DATA_TYPE_*.
DATA_TYPE_REPORT = 1
DATA_TYPE_CONFIG = 2
DATA_TYPE_LARGE  = 3

# Max binary bytes per AT#DATA chunk over UART (must match firmware's
# AT_DATA_PAYLOAD_MAX). The radio still ships SEND_DATA_MAX = 180 B per
# radio chunk; firmware splits each AT chunk into 1..N radio chunks when
# forwarding.
#
# Wire format (quoted hex fields):
#   AT#DATA="<id>","<page>","<chunk>","<crc32>","<payload_hex>"
# At 450 B payload: 'AT#DATA=' (8) + framing 30 + 900 payload hex + CR
# = 939 chars, within SLM_UART_AT_COMMAND_LEN (1024).
DATA_CHUNK_MAX = 450
LARGE_CHUNKS_PER_PAGE = 20  # matches large_data.c


def parse_sn_hex(s: str) -> int:
    """Parse a 64-bit serial number written as a 0x-prefixed hex string.

    The firmware addresses devices by SN as an 8-byte big-endian field, so
    we keep the input format strict: must start with '0x', be hex, and fit
    in 64 bits.
    """
    s = s.strip()
    if not s.lower().startswith("0x"):
        raise ValueError(f"SN must be 0x-prefixed hex (got {s!r})")
    try:
        value = int(s, 16)
    except ValueError as e:
        raise ValueError(f"SN is not valid hex: {s!r}") from e
    if value < 0 or value >= (1 << 64):
        raise ValueError(f"SN out of range for uint64: {s!r}")
    return value


class ATClient:
    def __init__(self, port: str, baud: int = DEFAULT_BAUD, timeout_s: float = DEFAULT_TIMEOUT_S,
                 rtscts: bool = True):
        # The firmware sets hw-flow-control on uart1, so the host must also
        # use RTS/CTS — otherwise the device may withhold TX waiting on CTS.
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1, rtscts=rtscts)
        self.timeout_s = timeout_s
        # Drop anything queued from a previous session.
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def send_raw(self, line: str) -> list[str]:
        """Send a single AT line and return any response lines collected.

        Protocol contract (no trailing OK): a command is considered
        SUCCESSFUL unless the device sends an 'ERROR' line. Queries reply
        with a '#...:' data line; commands that are silent on success return
        an empty list. Use response_is_error() to test the result.
        """
        if not line.endswith("\r"):
            line = line + "\r"
        self.ser.write(line.encode("ascii", errors="replace"))
        self.ser.flush()
        return self._read_response()

    def _read_response(self) -> list[str]:
        """Collect response lines for this command.

        Since there is no OK terminator, we can't wait for one. Instead:
          - Wait up to self.timeout_s for the FIRST byte of a response.
          - Once bytes arrive, keep reading until the line goes quiet for
            QUIET_GAP_S (the device finished its response).
          - If 'ERROR' is seen, return immediately.
          - If nothing arrives within the timeout, return [] (success for
            commands that are silent on success).
        """
        QUIET_GAP_S = 0.05
        deadline = time.monotonic() + self.timeout_s
        buf = bytearray()
        lines: list[str] = []
        got_anything = False
        last_rx = None

        while True:
            chunk = self.ser.read(64)
            if chunk:
                got_anything = True
                last_rx = time.monotonic()
                buf.extend(chunk)
                # Split on CR, LF, or CRLF — keep any partial trailing line in buf.
                text = buf.decode("ascii", errors="replace")
                parts = text.replace("\r\n", "\n").replace("\r", "\n").split("\n")
                buf = bytearray(parts[-1], "ascii")
                for piece in parts[:-1]:
                    piece = piece.strip()
                    if not piece:
                        continue
                    lines.append(piece)
                    if piece == "ERROR":
                        return lines
            else:
                now = time.monotonic()
                if got_anything:
                    # Response started; stop once the line has been quiet
                    # long enough that the device is clearly done.
                    if last_rx is not None and (now - last_rx) >= QUIET_GAP_S:
                        # Flush any complete trailing line left in buf.
                        tail = buf.decode("ascii", errors="replace").strip()
                        if tail:
                            lines.append(tail)
                        return lines
                elif now >= deadline:
                    # Nothing came back at all → silent success.
                    return lines
                time.sleep(0.01)

    @staticmethod
    def response_is_error(lines: list[str]) -> bool:
        """True if the device reported ERROR for the command."""
        return any(l == "ERROR" for l in lines)

    def send_data(
        self,
        sn: int,
        data_type: int,
        data_id: int,
        payload: bytes,
        chunk_size: int = DATA_CHUNK_MAX,
        verbose: bool = True,
    ) -> None:
        """Push a binary payload to a remote device via AT#DATAINIT + AT#DATA.

        Two phases:

        1. AT#DATAINIT="<SN>","<TYPE>","<ID>","<LEN>","<CRC>"
           all fields uppercase hex; announces + validates the transfer.

        2. AT#DATA="<ID>","<PAGE>","<CHUNK>","<CRC>","<DATA>"
           one or more chunks, ≤450 B payload each at the UART layer. CRC is
           crc32_ieee of just that chunk's payload; on mismatch the firmware
           returns ERROR and the host should retry the chunk.

        Chunk layout by data_type:
            CONFIG (2): one chunk only, page=0 chunk=0  (≤128 B for the
                        config payload itself; firmware enforces).
            REPORT (1): one chunk only, page=0 chunk=0  (≤450 B at AT layer;
                        firmware DATA_MAX_TRANSFER_SIZE = 256 B further caps).
            LARGE  (3): pages of 20 chunks each, page=0..N-1 chunk=0..19.

        Success is implicit (no OK is sent); only an 'ERROR' line means
        failure. Raises RuntimeError if the device returns ERROR.
        """
        if not (0 <= sn < (1 << 64)):
            raise ValueError(f"sn out of range: {sn}")
        if not (0 <= data_type < (1 << 8)):
            raise ValueError(f"data_type out of range: {data_type}")
        if not (0 <= data_id < (1 << 16)):
            raise ValueError(f"data_id out of range: {data_id}")
        if len(payload) >= (1 << 16):
            raise ValueError(f"payload too large for uint16 length: {len(payload)}")
        if chunk_size <= 0 or chunk_size > DATA_CHUNK_MAX:
            raise ValueError(
                f"chunk_size must be 1..{DATA_CHUNK_MAX} (got {chunk_size})"
            )

        crc32 = zlib.crc32(payload) & 0xFFFFFFFF

        # Phase 1 — DATAINIT.
        # Format: AT#DATAINIT="<SN>","<TYPE>","<ID>","<LEN>","<CRC>"
        # Every field is an uppercase hex string (no 0x prefix).
        cmd = (
            'AT#DATAINIT='
            f'"{sn:016X}",'
            f'"{data_type:02X}",'
            f'"{data_id:04X}",'
            f'"{len(payload):04X}",'
            f'"{crc32:08X}"'
        )
        if verbose:
            print(f">>> {cmd}")
        lines = self.send_raw(cmd)
        for line in lines:
            if verbose:
                print(line)
        if self.response_is_error(lines):
            raise RuntimeError(f"AT#DATAINIT failed: {lines!r}")

        if not payload:
            if verbose:
                print("(no payload bytes — DATAINIT-only)")
            return

        # Phase 2 — chunked payload with page/chunk numbering.
        total = len(payload)
        sent = 0
        # global_chunk = chunks-since-start; page/chunk derived from it.
        global_chunk = 0
        while sent < total:
            n = min(chunk_size, total - sent)
            chunk = payload[sent : sent + n]
            page  = global_chunk // LARGE_CHUNKS_PER_PAGE
            chunk_no = global_chunk %  LARGE_CHUNKS_PER_PAGE

            # Per-type layout sanity (firmware also enforces).
            if data_type == DATA_TYPE_CONFIG and global_chunk > 0:
                raise ValueError(
                    f"CONFIG payload too large for one chunk "
                    f"(max {DATA_CHUNK_MAX} B, got {len(payload)} B)"
                )
            if data_type == DATA_TYPE_REPORT and global_chunk > 0:
                raise ValueError(
                    f"REPORT payload too large for one chunk "
                    f"(max {DATA_CHUNK_MAX} B, got {len(payload)} B)"
                )

            chunk_crc32 = zlib.crc32(chunk) & 0xFFFFFFFF
            # Format: AT#DATA="<ID>","<PAGE>","<CHUNK>","<CRC>","<DATA>"
            # All fields uppercase hex strings; DATA is the hex payload.
            cmd = (
                'AT#DATA='
                f'"{data_id:04X}",'
                f'"{page:02X}",'
                f'"{chunk_no:02X}",'
                f'"{chunk_crc32:08X}",'
                f'"{chunk.hex().upper()}"'
            )

            if verbose:
                print(f">>> AT#DATA=\"{data_id:04X}\",\"{page:02X}\","
                      f"\"{chunk_no:02X}\",\"{chunk_crc32:08X}\",<{n} B hex>  "
                      f"[{sent + n}/{total}]")
            lines = self.send_raw(cmd)
            for line in lines:
                if verbose:
                    print(line)
            if self.response_is_error(lines):
                raise RuntimeError(
                    f"AT#DATA page={page} chunk={chunk_no} failed at "
                    f"offset {sent}: {lines!r}"
                )
            sent += n
            global_chunk += 1


def autodetect_port() -> str | None:
    """Best-effort port pick: prefer entries whose description mentions
    nRF/JLink/Segger, otherwise return the first available."""
    candidates = list(list_ports.comports())
    if not candidates:
        return None
    preferred = [p for p in candidates if any(k in (p.description or "").lower()
                                              for k in ("nrf", "jlink", "j-link", "segger"))]
    return (preferred[0] if preferred else candidates[0]).device


def run_oneshot(client: ATClient, commands: Iterable[str]) -> int:
    rc = 0
    for cmd in commands:
        cmd = cmd.strip()
        if not cmd:
            continue
        print(f">>> {cmd}")
        lines = client.send_raw(cmd)
        for line in lines:
            print(line)
        if client.response_is_error(lines):
            rc = 1
        elif not lines:
            # No reply line and no ERROR → silent success.
            print("(ok)")
    return rc


def run_repl(client: ATClient) -> int:
    print("hello_dect AT console — type 'quit' or Ctrl-D/Ctrl-C to exit")
    print("Useful commands: AT, AT#VERSION?, AT#DEVTYPE?, AT#DEVID?, AT#SN?, AT#HOP?, AT#REBOOT")
    while True:
        try:
            line = input("AT> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if not line:
            continue
        if line.lower() in ("quit", "exit"):
            return 0
        lines = client.send_raw(line)
        if lines:
            for resp in lines:
                print(resp)
        else:
            # No reply and no ERROR → command accepted silently.
            print("(ok)")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", help="serial port (e.g. COM7 or /dev/ttyACM0); auto-detect if omitted")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"baud rate (default {DEFAULT_BAUD})")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="per-command timeout in seconds")
    parser.add_argument("--no-rtscts", action="store_true", help="disable RTS/CTS flow control (firmware default has it ON)")
    parser.add_argument("--list", action="store_true", help="list available serial ports and exit")

    # Data push: announce via AT#DATAINIT, then stream chunks via AT#DATA<hex>.
    # The SN identifies the destination device on the mesh. data_id defaults
    # to 1 — override with --data-id if needed.
    send_group = parser.add_mutually_exclusive_group()
    send_group.add_argument("--send-config", metavar="SN_HEX",
                            help="push a CONFIG payload to the device with this 0x-prefixed SN")
    send_group.add_argument("--send-report", metavar="SN_HEX",
                            help="push a REPORT payload to the device with this 0x-prefixed SN")
    send_group.add_argument("--send-large", metavar="SN_HEX",
                            help="push a LARGE payload to the device with this 0x-prefixed SN")
    parser.add_argument("--file", metavar="PATH",
                        help="binary payload file (required when using --send-*)")
    parser.add_argument("--data-id", type=int, default=1,
                        help="data_id field for the transfer (default: 1)")
    parser.add_argument("--chunk-size", type=int, default=DATA_CHUNK_MAX,
                        help=f"bytes per AT#DATA chunk (default: {DATA_CHUNK_MAX})")

    parser.add_argument("commands", nargs="*", help="AT commands to run (REPL if none given)")
    args = parser.parse_args(argv)

    if args.list:
        for p in list_ports.comports():
            print(f"{p.device}\t{p.description}")
        return 0

    port = args.port or autodetect_port()
    if not port:
        sys.stderr.write("error: no serial port specified and none found via auto-detect.\n")
        return 2

    try:
        client = ATClient(port, baud=args.baud, timeout_s=args.timeout, rtscts=not args.no_rtscts)
    except serial.SerialException as e:
        sys.stderr.write(f"error: cannot open {port}: {e}\n")
        return 2

    # Note: the firmware does not echo and does not send OK on success — a
    # command is treated as successful unless it returns an 'ERROR' line.

    # Data-push mode short-circuits the REPL / one-shot paths.
    send_sn_hex = args.send_config or args.send_report or args.send_large
    if send_sn_hex:
        if not args.file:
            sys.stderr.write("error: --send-* requires --file <PATH>\n")
            client.close()
            return 2
        try:
            sn = parse_sn_hex(send_sn_hex)
        except ValueError as e:
            sys.stderr.write(f"error: {e}\n")
            client.close()
            return 2
        try:
            payload = Path(args.file).read_bytes()
        except OSError as e:
            sys.stderr.write(f"error: cannot read {args.file}: {e}\n")
            client.close()
            return 2

        if args.send_config:
            dtype = DATA_TYPE_CONFIG
        elif args.send_report:
            dtype = DATA_TYPE_REPORT
        else:
            dtype = DATA_TYPE_LARGE

        try:
            client.send_data(sn, dtype, args.data_id, payload,
                             chunk_size=args.chunk_size)
        except (RuntimeError, TimeoutError, ValueError) as e:
            sys.stderr.write(f"error: {e}\n")
            return 1
        finally:
            client.close()
        return 0

    try:
        if args.commands:
            return run_oneshot(client, args.commands)
        return run_repl(client)
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
