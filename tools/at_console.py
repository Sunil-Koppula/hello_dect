#!/usr/bin/env python3
"""AT command client for the hello_dect firmware (Gateway / Anchor / Sensor).

The firmware exposes an AT-style line protocol on uart1 (see
src/slm_at_main.c). Wire format:

    host -> device:   AT#CMD?<CR>                          (query)
                      AT#CMD="<arg1>","<arg2>"...<CR>      (set / push)
    device -> host:   <CR><LF>#TAG: value<CR><LF>          (data line, if any)
                      <CR><LF>OK<CR><LF>                   (some commands)
                      <CR><LF>ERROR<CR><LF>                (on failure)

Success is implicit for data commands (AT#CONFIG): the device replies with
a '#TAG: …' line and no OK. Only 'ERROR' indicates failure.

Examples:
    python at_console.py --port COM7                       # interactive REPL
    python at_console.py --port COM7 AT#DEVTYPE?           # one-shot
    python at_console.py --port /dev/ttyACM0 AT#VERSION? AT#HOP?

    # Set the device serial number (16 hex chars, quoted):
    python at_console.py --port COM26 'AT#SN="00CAFE00DEADBEEF"'

    # Push a config payload to a mesh device, addressed by its SN:
    python at_console.py --port COM26 \\
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

# Max config payload size in bytes — must match firmware MAX_CONFIG_SIZE
# (see protocol.h). The firmware's AT#CONFIG handler enforces this.
MAX_CONFIG_SIZE = 128


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

    def send_config(
        self,
        sn: int,
        data_id: int,
        payload: bytes,
        verbose: bool = True,
    ) -> None:
        """Push a CONFIG payload to a mesh device via a single AT#CONFIG line.

        Wire format (all fields uppercase hex inside quotes):
            AT#CONFIG="<SN:16>","<ID:4>","<LEN:4>","<CRC32:8>","<DATA hex>"

        The firmware (gateway only) validates the SN against its mesh table,
        allocates a config slot, verifies the CRC, persists the payload, and
        config_tick() drives the radio TX from there.

        Success is implicit: only an 'ERROR' line means failure. The device
        also emits a '#CONFIG: …' acknowledgement line on success.
        """
        if not (0 <= sn < (1 << 64)):
            raise ValueError(f"sn out of range: {sn}")
        if not (0 <= data_id < (1 << 16)):
            raise ValueError(f"data_id out of range: {data_id}")
        if not (0 < len(payload) <= MAX_CONFIG_SIZE):
            raise ValueError(
                f"payload must be 1..{MAX_CONFIG_SIZE} bytes (got {len(payload)})"
            )

        crc32 = zlib.crc32(payload) & 0xFFFFFFFF
        cmd = (
            'AT#CONFIG='
            f'"{sn:016X}",'
            f'"{data_id:04X}",'
            f'"{len(payload):04X}",'
            f'"{crc32:08X}",'
            f'"{payload.hex().upper()}"'
        )
        if verbose:
            print(f">>> AT#CONFIG=\"{sn:016X}\",\"{data_id:04X}\","
                  f"\"{len(payload):04X}\",\"{crc32:08X}\",<{len(payload)} B hex>")
        lines = self.send_raw(cmd)
        for line in lines:
            if verbose:
                print(line)
        if self.response_is_error(lines):
            raise RuntimeError(f"AT#CONFIG failed: {lines!r}")


# ---------------------------------------------------------------------------
# Downstream-MCU emulation: act as an AT *server* on the UART, accepting
# AT#CONFIG lines emitted by a hello_dect sensor (see config.c, sensor branch
# of config_tick). Verify CRC32 and reply OK / ERROR.
# ---------------------------------------------------------------------------

def _extract_quoted_fields(line: str) -> list[str] | None:
    """Return the list of strings between consecutive ""..."" pairs in 'line'.

    Mirrors the firmware's field_get() semantics: scans for matched '"'
    delimiters in order. Returns None if any quote is unbalanced.
    """
    fields: list[str] = []
    i = 0
    n = len(line)
    while i < n:
        # Find next opening quote.
        a = line.find('"', i)
        if a < 0:
            break
        b = line.find('"', a + 1)
        if b < 0:
            return None  # unbalanced
        fields.append(line[a + 1 : b])
        i = b + 1
    return fields


def _handle_at_config_line(line: str) -> tuple[bool, str, dict | None]:
    """Parse one AT#CONFIG line and verify it.

    Wire format:
        AT#CONFIG="<SN>","<ID>","<LEN>","<CRC32>","<DATA hex>"

    Returns (ok, summary, parsed):
      ok       — True on a fully-validated config.
      summary  — human-readable one-liner (success or failure reason).
      parsed   — {'sn', 'data_id', 'data_len', 'crc32', 'payload'} on success,
                 None on failure.
    """
    fields = _extract_quoted_fields(line)
    if fields is None or len(fields) < 5:
        return False, f"malformed: expected 5 quoted fields, got {fields!r}", None

    sn_hex, id_hex, len_hex, crc_hex, data_hex = fields[:5]
    try:
        sn        = int(sn_hex, 16)
        data_id   = int(id_hex, 16)
        data_len  = int(len_hex, 16)
        crc32     = int(crc_hex, 16)
    except ValueError as e:
        return False, f"bad hex in header: {e}", None

    if len(data_hex) != data_len * 2:
        return (False,
                f"len mismatch: header says {data_len} B "
                f"({data_len * 2} hex chars), got {len(data_hex)} hex chars",
                None)

    try:
        payload = bytes.fromhex(data_hex)
    except ValueError as e:
        return False, f"bad hex in payload: {e}", None

    calc = zlib.crc32(payload) & 0xFFFFFFFF
    if calc != crc32:
        return (False,
                f"CRC32 mismatch: header 0x{crc32:08X}, calc 0x{calc:08X}",
                None)

    summary = (f"sn=0x{sn:016X} id={data_id} len={data_len} "
               f"crc=0x{crc32:08X}\n  payload: {payload.hex().upper()}")
    parsed = {
        "sn": sn, "data_id": data_id, "data_len": data_len,
        "crc32": crc32, "payload": payload,
    }
    return True, summary, parsed


def run_downstream(client: "ATClient") -> int:
    """Continuously read lines from the device and respond to AT#CONFIG.

    Acts as the application MCU on the sensor's downstream UART:
      - Wait for incoming complete lines (CR/LF terminated).
      - For each AT#CONFIG line: parse + CRC-verify, then reply OK or ERROR.
      - Other AT#... lines: reply ERROR (we only implement #CONFIG here).
      - Non-AT lines: log them and ignore (don't reply).

    Ctrl-C exits cleanly.
    """
    print("hello_dect downstream-MCU emulator — Ctrl-C to exit")
    print(f"Listening on {client.ser.port} @ {client.ser.baudrate} baud")
    print("Replies OK after CRC32 verification, ERROR on any failure.\n")

    buf = bytearray()
    try:
        while True:
            chunk = client.ser.read(256)
            if not chunk:
                time.sleep(0.005)
                continue
            buf.extend(chunk)
            # Split on CR/LF/CRLF; keep the partial last line in buf.
            text = buf.decode("ascii", errors="replace")
            parts = text.replace("\r\n", "\n").replace("\r", "\n").split("\n")
            buf = bytearray(parts[-1], "ascii")

            for raw in parts[:-1]:
                line = raw.strip()
                if not line:
                    continue
                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] <<< {line}")

                if line.startswith("AT#CONFIG"):
                    ok, summary, parsed = _handle_at_config_line(line)
                    if ok:
                        # Ack with which (sn, data_id) was accepted, then OK.
                        ack = (
                            f'\r\n#CONFIG: "{parsed["sn"]:016X}",'
                            f'"{parsed["data_id"]:04X}"\r\n'
                        ).encode("ascii")
                        print(f"           OK — {summary}")
                        client.ser.write(ack)
                        client.ser.write(b"\r\nOK\r\n")
                    else:
                        print(f"           ERROR — {summary}")
                        client.ser.write(b"\r\nERROR\r\n")
                    client.ser.flush()
                elif line.startswith("AT"):
                    print("           (unknown AT command, replying ERROR)")
                    client.ser.write(b"\r\nERROR\r\n")
                    client.ser.flush()
                # else: silent log of garbage / partial bytes
    except KeyboardInterrupt:
        print("\n(exit)")
        return 0


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

    # Single-line config push via AT#CONFIG. The SN identifies the destination
    # device on the mesh. data_id is an application-level id (default 1).
    parser.add_argument("--send-config", metavar="SN_HEX",
                        help="push a CONFIG payload to the device with this 0x-prefixed SN")
    parser.add_argument("--file", metavar="PATH",
                        help="binary payload file (required with --send-config)")
    parser.add_argument("--data-id", type=lambda s: int(s, 0), default=1,
                        help="data_id field for the transfer (default: 1)")

    # Downstream-MCU emulator: listen on the UART for AT#CONFIG lines emitted
    # by the sensor's config_tick(), verify CRC32, reply OK/ERROR.
    parser.add_argument("--downstream", action="store_true",
                        help="act as the downstream application MCU: listen for "
                             "AT#CONFIG lines and reply OK/ERROR after verifying CRC32")

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

    # Downstream-MCU emulator mode short-circuits everything else.
    if args.downstream:
        if args.send_config or args.commands:
            sys.stderr.write("error: --downstream cannot be combined with "
                             "--send-config or positional commands\n")
            client.close()
            return 2
        try:
            return run_downstream(client)
        finally:
            client.close()

    # Config-push mode short-circuits the REPL / one-shot paths.
    if args.send_config:
        if not args.file:
            sys.stderr.write("error: --send-config requires --file <PATH>\n")
            client.close()
            return 2
        try:
            sn = parse_sn_hex(args.send_config)
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

        try:
            client.send_config(sn, args.data_id, payload)
        except (RuntimeError, ValueError) as e:
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
