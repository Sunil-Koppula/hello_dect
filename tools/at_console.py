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

Requires: pyserial  (pip install pyserial)
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.stderr.write("error: pyserial not installed. Run: pip install pyserial\n")
    sys.exit(1)


DEFAULT_BAUD = 1000000  # uart1 current-speed in nrf9151dk_nrf9151_ns.overlay
DEFAULT_TIMEOUT_S = 2.0


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
        """Send a single AT line and return the response lines (without the
        terminator). Raises TimeoutError if no OK/ERROR arrives in time."""
        if not line.endswith("\r"):
            line = line + "\r"
        self.ser.write(line.encode("ascii", errors="replace"))
        self.ser.flush()
        return self._read_response()

    def _read_response(self) -> list[str]:
        """Collect lines until OK or ERROR. Returns the data lines plus the
        terminator as the last element."""
        deadline = time.monotonic() + self.timeout_s
        buf = bytearray()
        lines: list[str] = []

        while time.monotonic() < deadline:
            chunk = self.ser.read(64)
            if chunk:
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
                    if piece in ("OK", "ERROR"):
                        return lines
            else:
                # No data this poll — short pause to avoid busy-looping.
                time.sleep(0.01)

        raise TimeoutError(f"no OK/ERROR within {self.timeout_s:.1f}s; got {lines!r}")


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
        try:
            lines = client.send_raw(cmd)
        except TimeoutError as e:
            print(f"!!! {e}")
            rc = 2
            continue
        for line in lines:
            print(line)
        if lines and lines[-1] == "ERROR":
            rc = 1
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
        try:
            lines = client.send_raw(line)
        except TimeoutError as e:
            print(f"!!! {e}")
            continue
        for resp in lines:
            print(resp)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", help="serial port (e.g. COM7 or /dev/ttyACM0); auto-detect if omitted")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"baud rate (default {DEFAULT_BAUD})")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="per-command timeout in seconds")
    parser.add_argument("--no-rtscts", action="store_true", help="disable RTS/CTS flow control (firmware default has it ON)")
    parser.add_argument("--list", action="store_true", help="list available serial ports and exit")
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

    # Disable device-side echo so responses are unambiguous.
    try:
        client.send_raw("ATE0")
    except TimeoutError:
        pass  # device may not respond if it was mid-boot — keep going

    try:
        if args.commands:
            return run_oneshot(client, args.commands)
        return run_repl(client)
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
