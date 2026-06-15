#!/usr/bin/env python3
"""Tkinter GUI for the gateway-side host (config pusher + report viewer).

The counterpart to sensor_gui.py. Where sensor_gui emulates the application
MCU *below* a sensor, this tool emulates the host *above* a gateway:

  - PUSHES AT#CONFIG down to a target sensor SN. A form builds the
    sensor_config_structure_t (name "ESC21": command, fw_version,
    battery_level_min, sleep_time_sec, temp/hum max/min, flags), CRC32-frames
    it, sends AT#CONFIG=..., and shows the gateway's #CONFIG/OK/ERROR reply.

  - RECEIVES AT#REPORT coming up from sensors via the gateway. Each line is
    CRC-verified and the sensor_data_structure_t payload (3300 temp/humidity,
    battery, alarms) is decoded into a live table + log. The tool replies
    '#REPORT: ...' + OK on success, ERROR on failure — matching what the
    gateway's dispatch expects (cmd_report_ack + OK releases the slot).

Reuses sensor_sim.py for all encode/decode/framing. tkinter ships with CPython.

Usage:
    python gateway_gui.py --port COM31
    python gateway_gui.py                 # auto-detect port

    # Provision the gateway's own SN on startup, pre-fill a destination sensor:
    python gateway_gui.py --port COM31 --gateway-sn 00F91200DEADBEEF \\
        --sn AB0102030405

--gateway-sn provisions the device on the port (the gateway) via AT#SN?/AT#SN=;
--sn is only the default DESTINATION sensor SN for the config form.
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
import zlib

import tkinter as tk
from tkinter import ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.stderr.write("error: pyserial not installed. Run: pip install pyserial\n")
    sys.exit(1)

import sensor_sim as sim


# ---------------------------------------------------------------------------
# Worker: owns the serial port, runs the report-reader loop, sends configs.
# ---------------------------------------------------------------------------

class GatewayWorker:
    def __init__(self, io: sim.SerialIO):
        self.io = io
        self.stop_evt = threading.Event()
        self.events: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self._wlock = threading.Lock()

    def log(self, msg: str) -> None:
        self.events.put(("log", f"[{time.strftime('%H:%M:%S')}] {msg}"))

    # -- config push -----------------------------------------------------
    def send_config(self, sn: int, data_id: int, payload: bytes) -> None:
        """Send AT#CONFIG and report the gateway's reply via the event queue."""
        line = sim.build_at_config(sn, data_id, payload)
        crc = zlib.crc32(payload) & 0xFFFFFFFF
        self.log(f">>> AT#CONFIG sn=0x{sn:016X} id={data_id} len={len(payload)} crc=0x{crc:08X}")
        try:
            with self._wlock:
                self.io.write(line.encode("ascii", errors="replace"))
        except Exception as e:
            self.log(f"serial write failed: {e!r}")

    # -- report reader ---------------------------------------------------
    def reader_loop(self) -> None:
        buf = bytearray()
        while not self.stop_evt.is_set():
            try:
                chunk = self.io.ser.read(256)
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
                if line.startswith("AT#REPORT="):
                    self._handle_report(line)
                elif line in ("OK", "ERROR") or line.startswith("#CONFIG"):
                    # Replies to our AT#CONFIG push.
                    self.log(f"<<< {line}")
                else:
                    self.log(f"<<< {line}")

    def _handle_report(self, line: str) -> None:
        ok, summary, parsed = sim.parse_at_report(line)
        if not ok:
            with self._wlock:
                self.io.write(b"\r\nERROR\r\n")
            self.log(f"<<< AT#REPORT rejected — {summary}")
            return
        # Reply #REPORT ack + OK (mirrors at_console; gateway routes #REPORT to
        # cmd_report_ack, then OK releases the slot).
        with self._wlock:
            self.io.write(f'\r\n#REPORT: "{parsed["sn"]:016X}",'
                          f'"{parsed["data_id"]:04X}"\r\n'.encode("ascii"))
            self.io.write(b"\r\nOK\r\n")
        decoded = sim.decode_report_payload(parsed["payload"])
        self.log(f"<<< AT#REPORT — {summary}")
        self.events.put(("report", {
            "ts": time.strftime("%H:%M:%S"),
            "sn": f"{parsed['sn']:016X}",
            "priority": parsed["priority"],
            "timestamp": parsed.get("timestamp"),
            "decoded": decoded,
        }))

    def start(self) -> None:
        threading.Thread(target=self.reader_loop, name="reader", daemon=True).start()

    def stop(self) -> None:
        self.stop_evt.set()


# ---------------------------------------------------------------------------
# Tk GUI
# ---------------------------------------------------------------------------

class GatewayGUI(tk.Tk):
    def __init__(self, worker: GatewayWorker, default_sn: str = ""):
        super().__init__()
        self.worker = worker
        self.title("hello_dect gateway host — config push / report view")
        self.geometry("1080x720")
        self.configure(padx=10, pady=10)

        self._build_config_form(default_sn)
        self._build_report_table()
        self._build_log()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(100, self._drain_events)

    # -- config form -----------------------------------------------------
    def _build_config_form(self, default_sn: str):
        f = ttk.LabelFrame(self, text="Config (manual push + auto-reply to every report)", padding=8)
        f.pack(fill="x", pady=4)
        ttk.Label(f, text="Auto-config is ON: these values are pushed back to "
                  "every reporting sensor's SN. 'Dest SN' is used only for the "
                  "manual Send button.", foreground="gray", wraplength=820,
                  justify="left").grid(row=11, column=0, columnspan=4, sticky="w", pady=(6, 0))

        self.e = {}

        def field(row, col, label, key, default, width=12):
            ttk.Label(f, text=label).grid(row=row, column=col * 2, sticky="w", padx=(4, 2), pady=2)
            e = ttk.Entry(f, width=width)
            e.insert(0, str(default))
            e.grid(row=row, column=col * 2 + 1, sticky="w", padx=(0, 8))
            self.e[key] = e

        field(0, 0, "Dest SN (hex):", "sn", default_sn, width=18)
        field(0, 1, "Data ID:", "data_id", "1", width=8)

        # command checkboxes (NO is implicit when none checked).
        cmdf = ttk.Frame(f)
        cmdf.grid(row=1, column=0, columnspan=4, sticky="w", pady=4)
        ttk.Label(cmdf, text="Command:").pack(side="left")
        self.v_demo = tk.BooleanVar()
        self.v_reset = tk.BooleanVar()
        ttk.Checkbutton(cmdf, text="DEMO_MODE", variable=self.v_demo).pack(side="left", padx=6)
        ttk.Checkbutton(cmdf, text="RESET", variable=self.v_reset).pack(side="left", padx=6)
        ttk.Label(cmdf, text="(none = NO)").pack(side="left", padx=6)

        field(2, 0, "FW version:", "fw", "1", width=8)
        field(2, 1, "Battery min %:", "bat_min", "30", width=8)
        field(3, 0, "Sleep time s:", "sleep", "600", width=8)
        field(3, 1, "Random:", "rand", "0", width=8)
        field(4, 0, "Temp1 Max C:", "tmax", "40")
        field(4, 1, "Temp1 Min C:", "tmin", "-10")
        field(5, 0, "Hum1 Max %:", "hmax", "80")
        field(5, 1, "Hum1 Min %:", "hmin", "20")
        field(6, 0, "Temp2 Max C:", "tmax2", "40")
        field(6, 1, "Temp2 Min C:", "tmin2", "-10")
        field(7, 0, "Hum2 Max %:", "hmax2", "80")
        field(7, 1, "Hum2 Min %:", "hmin2", "20")
        field(8, 0, "Ultrasound lvl max:", "us_max", "100")
        field(8, 1, "Ultrasound ctr freq:", "us_freq", "0")
        field(9, 0, "Vibration lvl max:", "vib_max", "500")

        self.v_notenc = tk.BooleanVar(value=True)
        ttk.Checkbutton(f, text="NOT_ENCRYPTED (0x01)", variable=self.v_notenc).grid(
            row=10, column=0, columnspan=2, sticky="w", pady=4)
        ttk.Button(f, text="Send AT#CONFIG", command=self._send_config).grid(
            row=10, column=3, sticky="e")

    def _build_config_from_form(self, dest_sn_hex: str):
        """Build (sn, data_id, payload) from the current form fields, targeting
        dest_sn_hex. Returns None on a parse error (logged). Must be called on
        the Tk thread (reads widget values)."""
        try:
            sn = int(dest_sn_hex, 16)
            data_id = int(self.e["data_id"].get(), 0)
            command = (sim.CMD_DEMO_MODE if self.v_demo.get() else 0) | \
                      (sim.CMD_RESET if self.v_reset.get() else 0)
            payload = sim.build_config_payload(
                dest_sn_hex=dest_sn_hex,
                command=command,
                new_fw_version=int(self.e["fw"].get(), 0),
                battery_level_min=int(self.e["bat_min"].get()),
                sleep_time_sec=int(self.e["sleep"].get()),
                temp_max=int(self.e["tmax"].get()),
                temp_min=int(self.e["tmin"].get()),
                hum_max=int(self.e["hmax"].get()),
                hum_min=int(self.e["hmin"].get()),
                temp_max2=int(self.e["tmax2"].get()),
                temp_min2=int(self.e["tmin2"].get()),
                hum_max2=int(self.e["hmax2"].get()),
                hum_min2=int(self.e["hmin2"].get()),
                ultrasound_level_max=int(self.e["us_max"].get()),
                ultrasound_center_frequency=int(self.e["us_freq"].get()),
                vibration_level_max=int(self.e["vib_max"].get()),
                random_number=int(self.e["rand"].get()),
                config_flags=sim.FLAG_NOT_ENCRYPTED if self.v_notenc.get() else 0,
            )
        except ValueError as e:
            self.worker.log(f"config form error: {e}")
            return None
        return sn, data_id, payload

    def _send_config(self):
        built = self._build_config_from_form(self.e["sn"].get())
        if built is None:
            return
        sn, data_id, payload = built
        self.worker.send_config(sn, data_id, payload)

    # -- report table ----------------------------------------------------
    def _build_report_table(self):
        f = ttk.LabelFrame(self, text="Incoming Reports (AT#REPORT)", padding=8)
        f.pack(fill="both", expand=True, pady=4)
        cols = ("time", "uptime", "sn", "type", "batt", "t1", "h1", "t2", "h2",
                "us", "vib", "alarm", "prio", "fw")
        self.tree = ttk.Treeview(f, columns=cols, show="headings", height=10)
        widths = {"time": 70, "uptime": 95, "sn": 130, "type": 60, "batt": 50,
                  "t1": 65, "h1": 65, "t2": 65, "h2": 65, "us": 80, "vib": 80,
                  "alarm": 150, "prio": 45, "fw": 40}
        heads = {"time": "Recv Time", "uptime": "Dev Uptime", "sn": "SN", "type": "Type",
                 "batt": "Batt%", "t1": "Temp1 C", "h1": "Hum1 %", "t2": "Temp2 C",
                 "h2": "Hum2 %", "us": "US lvl/frq", "vib": "Vib lvl/frq",
                 "alarm": "Alarm", "prio": "Prio", "fw": "FW"}
        for c in cols:
            self.tree.heading(c, text=heads[c])
            self.tree.column(c, width=widths[c], anchor="w")
        vsb = ttk.Scrollbar(f, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=vsb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")
        self.tree.tag_configure("alarm", foreground="red")

    @staticmethod
    def _fmt_uptime(ms) -> str:
        """Format the device uptime (k_uptime_get ms) as HH:MM:SS.mmm."""
        if ms is None:
            return "—"
        s, msec = divmod(int(ms), 1000)
        h, rem = divmod(s, 3600)
        m, sec = divmod(rem, 60)
        return f"{h:02d}:{m:02d}:{sec:02d}.{msec:03d}"

    def _add_report_row(self, rep: dict):
        d = rep["decoded"] or {}
        prio = "—" if rep["priority"] is None else rep["priority"]
        uptime = self._fmt_uptime(rep.get("timestamp"))
        if "raw" in d:
            vals = (rep["ts"], uptime, rep["sn"], "?", "?", "?", "?", "?", "?", "?", "?",
                    f"decode err ({d.get('note','')})", prio, "?")
            alarmed = False
        else:
            vals = (rep["ts"], uptime, d["sn"] or rep["sn"], f"0x{d['report_type']:04X}",
                    d["battery_level"], d["temperature1"], d["humidity1"],
                    d["temperature2"], d["humidity2"],
                    f"{d['ultrasound_level']}/{d['ultrasound_frequency']}",
                    f"{d['vibration_level']}/{d['vibration_frequency']}",
                    d["alarm_flags"], prio, d["firmware_version"])
            alarmed = d["alarm_flags_raw"] != 0
        item = self.tree.insert("", "end", values=vals, tags=("alarm",) if alarmed else ())
        self.tree.see(item)
        # Cap to ~500 rows.
        kids = self.tree.get_children()
        if len(kids) > 500:
            self.tree.delete(kids[0])

    # -- log -------------------------------------------------------------
    def _build_log(self):
        f = ttk.LabelFrame(self, text="Event Log", padding=8)
        f.pack(fill="both", expand=True, pady=4)
        self.txt = tk.Text(f, height=8, font=("Consolas", 8), wrap="none")
        self.txt.pack(fill="both", expand=True)
        self.txt.configure(state="disabled")

    def _drain_events(self):
        try:
            while True:
                kind, data = self.worker.events.get_nowait()
                if kind == "log":
                    self.txt.configure(state="normal")
                    self.txt.insert("end", data + "\n")
                    self.txt.see("end")
                    if int(self.txt.index("end-1c").split(".")[0]) > 500:
                        self.txt.delete("1.0", "100.0")
                    self.txt.configure(state="disabled")
                elif kind == "report":
                    self._add_report_row(data)
                    self._auto_send_config(data)
        except queue.Empty:
            pass
        self.after(100, self._drain_events)

    def _auto_send_config(self, rep: dict):
        """On every received report, push the current form's config back to the
        reporting sensor's SN (always-on auto-config)."""
        dest_sn_hex = rep["sn"]          # 16-hex AT-header SN of the reporter
        built = self._build_config_from_form(dest_sn_hex)
        if built is None:
            return
        sn, data_id, payload = built
        self.worker.log(f"auto-config -> reporter SN 0x{sn:016X} (id={data_id})")
        self.worker.send_config(sn, data_id, payload)

    def _on_close(self):
        self.worker.stop()
        self.destroy()


# ---------------------------------------------------------------------------
# CLI / bootstrap
# ---------------------------------------------------------------------------

def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port", help="serial port (auto-detect if omitted)")
    p.add_argument("--baud", type=int, default=sim.DEFAULT_BAUD)
    p.add_argument("--timeout", type=float, default=sim.DEFAULT_TIMEOUT_S)
    p.add_argument("--no-rtscts", action="store_true")
    p.add_argument("--sn", default="", help="default DESTINATION sensor SN for the config form")
    p.add_argument("--gateway-sn", help="the gateway's own SN (hex); provisioned on "
                   "startup via AT#SN?/AT#SN= before the GUI opens")
    p.add_argument("--skip-sn-check", action="store_true",
                   help="skip the gateway SN provisioning handshake")
    p.add_argument("--packed", action="store_true")
    args = p.parse_args(argv)

    sim.PACKED = args.packed

    port = args.port or sim.autodetect_port()
    if not port:
        sys.stderr.write("error: no serial port given and none auto-detected.\n")
        return 2
    try:
        io = sim.SerialIO(port, args.baud, args.timeout, rtscts=not args.no_rtscts)
    except serial.SerialException as e:
        sys.stderr.write(f"error: cannot open {port}: {e}\n")
        return 2

    # Provision the gateway's own SN before opening the GUI (read AT#SN?, set
    # via AT#SN= if it differs, abort on failure). Runs synchronously here,
    # before the report-reader thread starts, so there's no ser.read() race.
    if args.gateway_sn and not args.skip_sn_check:
        try:
            gw_sn = int(args.gateway_sn, 16)
        except ValueError:
            sys.stderr.write(f"error: --gateway-sn not valid hex: {args.gateway_sn!r}\n")
            io.close()
            return 2
        if not sim.provision_sn(io, gw_sn):
            io.close()
            return 1

    worker = GatewayWorker(io)
    worker.start()

    gui = GatewayGUI(worker, default_sn=args.sn)
    try:
        gui.mainloop()
    finally:
        worker.stop()
        io.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
