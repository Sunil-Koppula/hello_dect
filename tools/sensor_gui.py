#!/usr/bin/env python3
"""Tkinter GUI for the hello_dect sensor simulator.

A windowed front-end over sensor_sim.py. It reuses that module's sensor model
and AT framing (SensorState, build_at_report, parse_at_config,
decode_config_payload, provision_sn, ...) and drives them from a Tk window
instead of the terminal dashboard — sidestepping all terminal-redraw issues.

Two background threads do the I/O (off the Tk main loop):
  - report loop : battery drain, reading regeneration, interval + alarm sends
  - config reader: receives AT#CONFIG pushes, replies OK/ERROR, applies them

The GUI shows live state and offers manual controls:
  - Send report now
  - Set battery %, set temperature / humidity (manual override of random gen)
  - Toggle a forced alarm

tkinter ships with CPython — no extra install needed.

Usage:
    python sensor_gui.py --port COM32 --sn AB0102030405
    python sensor_gui.py            # then pick the port in the connect bar
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
    from serial.tools import list_ports
    import serial
except ImportError:
    sys.stderr.write("error: pyserial not installed. Run: pip install pyserial\n")
    sys.exit(1)

import sensor_sim as sim


# ---------------------------------------------------------------------------
# Worker: owns the serial port + sensor state, runs the two background loops.
# Communicates with the GUI via a thread-safe event queue (the GUI polls it).
# ---------------------------------------------------------------------------

class SensorWorker:
    def __init__(self, state: sim.SensorState, io: sim.SerialIO, args):
        self.state = state
        self.io = io
        self.args = args
        self.stop_evt = threading.Event()
        self.events: "queue.Queue[tuple[str, str]]" = queue.Queue()

        # Manual overrides set from the GUI. When *_override is not None the
        # report loop uses it instead of the random/drained value, and reading
        # regeneration is suppressed while temp/hum are overridden.
        self.battery_override = None
        self.temp_override = None
        self.hum_override = None
        self.force_alarm = 0            # extra alarm bits OR-ed in
        self._send_now = threading.Event()
        # data_id auto-increments per report, wrapping at DATA_ID_MAX -> 1.
        self.data_id = max(1, min(sim.DATA_ID_MAX, args.data_id))

    # -- helpers ---------------------------------------------------------
    def log(self, msg: str) -> None:
        self.events.put(("log", f"[{time.strftime('%H:%M:%S')}] {msg}"))

    def request_send(self) -> None:
        self._send_now.set()

    # -- the AT#REPORT send ---------------------------------------------
    def _send(self, reason: str) -> None:
        st = self.state
        # Apply manual overrides before snapshotting.
        with st.lock:
            if self.battery_override is not None:
                st.battery = self.battery_override
            if self.temp_override is not None:
                st.temperature = self.temp_override
            if self.hum_override is not None:
                st.humidity = self.hum_override
        alarm = st.compute_alarm_flags() | self.force_alarm
        priority = sim.PRIO_ALARM if alarm else sim.PRIO_NORMAL
        payload = st.snapshot_payload(alarm)
        data_id = self.data_id
        line = sim.build_at_report(st.sn, data_id, payload, priority)
        try:
            self.io.write(line.encode("ascii", errors="replace"))
        except Exception as e:
            self.log(f"serial write failed: {e!r}")
            return
        self.data_id = sim.next_data_id(self.data_id)   # advance for next report
        with st.lock:
            st.last_alarm_flags = alarm
            st.last_priority = priority
            st.last_at_command = line.strip()
            st.last_response = f"sent ({reason}, id={data_id}, prio={priority})"
        crc = zlib.crc32(payload) & 0xFFFFFFFF
        self.log(f">>> AT#REPORT ({reason}) id={data_id} prio={priority} "
                 f"alarm=0x{alarm:04X}[{sim.alarm_text(alarm)}] crc=0x{crc:08X}")

    # -- report loop -----------------------------------------------------
    def report_loop(self) -> None:
        now = time.monotonic()
        next_battery = now + self.args.battery_period
        next_check = now + self.args.gen_period   # parameter-check (and regen) cadence
        next_report = now
        while not self.stop_evt.is_set():
            now = time.monotonic()

            if self.battery_override is None and now >= next_battery:
                self.state.drain_battery()
                next_battery += self.args.battery_period

            # Every gen_period: regenerate readings (unless overridden), then
            # CHECK PARAMETERS. If any alarm condition holds, send immediately.
            if now >= next_check:
                if self.temp_override is None and self.hum_override is None:
                    self.state.regen_readings()
                next_check += self.args.gen_period
                alarm = self.state.compute_alarm_flags() | self.force_alarm
                if alarm:
                    self._send("alarm-check")
                    next_report = now + self.state.sleep_time_sec

            # Manual "send report now" button.
            if self._send_now.is_set():
                self._send_now.clear()
                self._send("manual")
                next_report = now + self.state.sleep_time_sec

            # Scheduled interval report (normal periodic).
            if now >= next_report:
                self._send("interval")
                next_report = now + self.state.sleep_time_sec

            self.stop_evt.wait(0.2)

    # -- config reader ---------------------------------------------------
    def config_loop(self) -> None:
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
                if not line or not line.startswith("AT#CONFIG="):
                    continue
                ok, summary, parsed = sim.parse_at_config(line)
                if not ok:
                    self.io.write(b"\r\nERROR\r\n")
                    with self.state.lock:
                        self.state.last_at_command = f"<< {line}"
                        self.state.last_response = f"AT#CONFIG rejected — {summary}"
                    self.log(f"<<< AT#CONFIG rejected — {summary}")
                    continue
                self.io.write(f'\r\n#CONFIG: "{parsed["sn"]:016X}",'
                              f'"{parsed["data_id"]:04X}"\r\n'.encode("ascii"))
                self.io.write(b"\r\nOK\r\n")
                decoded = sim.decode_config_payload(parsed["payload"])
                notes = (self.state.apply_config(decoded)
                         if decoded and "raw" not in decoded else [])
                with self.state.lock:
                    self.state.last_at_command = f"<< {line}"
                    self.state.last_response = (f"AT#CONFIG accepted — {summary}"
                                                + (f"  | {', '.join(notes)}" if notes else ""))
                self.log(f"<<< AT#CONFIG accepted — {summary}"
                         + (f"  | {', '.join(notes)}" if notes else ""))

    def start(self) -> None:
        threading.Thread(target=self.report_loop, name="report", daemon=True).start()
        threading.Thread(target=self.config_loop, name="config", daemon=True).start()

    def stop(self) -> None:
        self.stop_evt.set()


# ---------------------------------------------------------------------------
# Tk GUI
# ---------------------------------------------------------------------------

class SensorGUI(tk.Tk):
    def __init__(self, worker: SensorWorker):
        super().__init__()
        self.worker = worker
        self.state_ = worker.state
        self.title(f"hello_dect sensor sim — SN {self.state_.sn:012X}")
        self.geometry("760x640")
        self.configure(padx=10, pady=10)

        self._build_header()
        self._build_config_panel()
        self._build_params_panel()
        self._build_controls()
        self._build_at_panel()
        self._build_log()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(250, self._refresh)        # periodic UI refresh
        self.after(100, self._drain_events)    # log/event pump

    # -- layout ----------------------------------------------------------
    def _section(self, title: str) -> ttk.LabelFrame:
        f = ttk.LabelFrame(self, text=title, padding=8)
        f.pack(fill="x", pady=4)
        return f

    def _build_header(self):
        f = self._section("Identity")
        self.v_sn = tk.StringVar()
        self.v_type = tk.StringVar()
        self.v_time = tk.StringVar()
        ttk.Label(f, textvariable=self.v_sn, font=("Consolas", 11, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(f, textvariable=self.v_type).grid(row=0, column=1, padx=20, sticky="w")
        ttk.Label(f, textvariable=self.v_time).grid(row=0, column=2, padx=20, sticky="w")

    def _build_config_panel(self):
        f = self._section("Current Config")
        self.v_cfg = {k: tk.StringVar() for k in
                      ("report_time", "bat_min", "tmax", "tmin", "hmax", "hmin", "flags", "ts")}
        rows = [
            ("Report time:", "report_time", "Min Battery Level:", "bat_min"),
            ("Temp Max:", "tmax", "Hum Max:", "hmax"),
            ("Temp Min:", "tmin", "Hum Min:", "hmin"),
        ]
        for r, (l1, k1, l2, k2) in enumerate(rows):
            ttk.Label(f, text=l1, width=16).grid(row=r, column=0, sticky="w")
            ttk.Label(f, textvariable=self.v_cfg[k1], width=12).grid(row=r, column=1, sticky="w")
            ttk.Label(f, text=l2, width=18).grid(row=r, column=2, sticky="w")
            ttk.Label(f, textvariable=self.v_cfg[k2], width=12).grid(row=r, column=3, sticky="w")
        ttk.Label(f, text="Config Flags:", width=16).grid(row=3, column=0, sticky="w")
        ttk.Label(f, textvariable=self.v_cfg["flags"]).grid(row=3, column=1, columnspan=3, sticky="w")
        ttk.Label(f, textvariable=self.v_cfg["ts"], foreground="gray").grid(row=4, column=0, columnspan=4, sticky="w")

    def _build_params_panel(self):
        f = self._section("Current Parameters")
        self.v_batt = tk.StringVar()
        self.v_temp = tk.StringVar()
        self.v_hum = tk.StringVar()
        self.v_alarm = tk.StringVar()
        ttk.Label(f, text="Battery:", width=12).grid(row=0, column=0, sticky="w")
        ttk.Label(f, textvariable=self.v_batt, width=14).grid(row=0, column=1, sticky="w")
        ttk.Label(f, text="Temp:", width=10).grid(row=0, column=2, sticky="w")
        ttk.Label(f, textvariable=self.v_temp, width=12).grid(row=0, column=3, sticky="w")
        ttk.Label(f, text="Hum:", width=8).grid(row=0, column=4, sticky="w")
        ttk.Label(f, textvariable=self.v_hum, width=12).grid(row=0, column=5, sticky="w")
        self.l_alarm = ttk.Label(f, textvariable=self.v_alarm, font=("Consolas", 10, "bold"))
        self.l_alarm.grid(row=1, column=0, columnspan=6, sticky="w", pady=(6, 0))

    def _build_controls(self):
        f = self._section("Manual Controls")
        # Battery override.
        ttk.Label(f, text="Battery %:").grid(row=0, column=0, sticky="w")
        self.e_batt = ttk.Entry(f, width=6)
        self.e_batt.grid(row=0, column=1, sticky="w")
        ttk.Button(f, text="Set", command=self._set_battery).grid(row=0, column=2, padx=4)
        ttk.Button(f, text="Auto", command=self._auto_battery).grid(row=0, column=3)

        # Temp / hum override.
        ttk.Label(f, text="Temp °C:").grid(row=1, column=0, sticky="w", pady=4)
        self.e_temp = ttk.Entry(f, width=8)
        self.e_temp.grid(row=1, column=1, sticky="w")
        ttk.Label(f, text="Hum %:").grid(row=1, column=2, sticky="w")
        self.e_hum = ttk.Entry(f, width=8)
        self.e_hum.grid(row=1, column=3, sticky="w")
        ttk.Button(f, text="Set readings", command=self._set_readings).grid(row=1, column=4, padx=4)
        ttk.Button(f, text="Auto", command=self._auto_readings).grid(row=1, column=5)

        # Force alarm + send now.
        self.v_force = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="Force alarm (TEMP1)", variable=self.v_force,
                        command=self._toggle_force).grid(row=2, column=0, columnspan=2, sticky="w", pady=4)
        ttk.Button(f, text="Send report now", command=self.worker.request_send).grid(row=2, column=4, columnspan=2, sticky="e")

    def _build_at_panel(self):
        f = self._section("Last AT Command / Response")
        self.v_at = tk.StringVar()
        self.v_resp = tk.StringVar()
        ttk.Label(f, textvariable=self.v_at, font=("Consolas", 8), wraplength=720,
                  justify="left").pack(anchor="w")
        ttk.Separator(f, orient="horizontal").pack(fill="x", pady=4)
        ttk.Label(f, textvariable=self.v_resp, font=("Consolas", 9)).pack(anchor="w")

    def _build_log(self):
        f = self._section("Event Log")
        self.txt = tk.Text(f, height=8, font=("Consolas", 8), wrap="none")
        self.txt.pack(fill="both", expand=True)
        self.txt.configure(state="disabled")

    # -- control callbacks ----------------------------------------------
    def _set_battery(self):
        try:
            v = max(0, min(100, int(self.e_batt.get())))
        except ValueError:
            return
        self.worker.battery_override = v
        self.worker.log(f"manual battery override = {v}%")

    def _auto_battery(self):
        self.worker.battery_override = None
        self.worker.log("battery override cleared (auto drain)")

    def _set_readings(self):
        try:
            t = float(self.e_temp.get()) if self.e_temp.get() else None
            h = float(self.e_hum.get()) if self.e_hum.get() else None
        except ValueError:
            return
        if t is not None:
            self.worker.temp_override = max(-327.0, min(327.0, t))
        if h is not None:
            self.worker.hum_override = max(0.0, min(100.0, h))
        self.worker.log(f"manual readings override temp={self.worker.temp_override} "
                        f"hum={self.worker.hum_override}")

    def _auto_readings(self):
        self.worker.temp_override = None
        self.worker.hum_override = None
        self.worker.log("readings override cleared (auto regen)")

    def _toggle_force(self):
        self.worker.force_alarm = sim.ALARM_TEMPERATURE1 if self.v_force.get() else 0
        self.worker.log(f"force alarm = {'on' if self.v_force.get() else 'off'}")

    # -- periodic refresh ------------------------------------------------
    def _refresh(self):
        st = self.state_
        with st.lock:
            sn, rtype = st.sn, st.last_report_type
            sleep_s, bat_min = st.sleep_time_sec, st.battery_level_min
            tmax, tmin, hmax, hmin = st.temp_max, st.temp_min, st.hum_max, st.hum_min
            cfg_flags, cfg_ts = st.config_flags, st.last_config_ts
            battery, temp, hum = st.battery, st.temperature, st.humidity
            alarm = st.last_alarm_flags
            at_cmd, resp = st.last_at_command, st.last_response

        dash = lambda v, s="": f"{v}{s}" if v is not None else "—"
        self.v_sn.set(f"SN: {sn:012X}  ({sn})")
        self.v_type.set(f"Type: 0x{rtype:04X}")
        self.v_time.set(time.strftime("%Y-%m-%d %H:%M:%S"))

        self.v_cfg["report_time"].set(f"{sleep_s}s")
        self.v_cfg["bat_min"].set(f"{bat_min}%")
        self.v_cfg["tmax"].set(dash(tmax, "C"))
        self.v_cfg["tmin"].set(dash(tmin, "C"))
        self.v_cfg["hmax"].set(dash(hmax, "%"))
        self.v_cfg["hmin"].set(dash(hmin, "%"))
        enc = "NOT_ENCRYPTED" if (cfg_flags & sim.FLAG_NOT_ENCRYPTED) else "ENCRYPTED"
        self.v_cfg["flags"].set(f"0x{cfg_flags:02X} ({enc})")
        self.v_cfg["ts"].set(f"updated {cfg_ts}" if cfg_ts else "defaults — no config yet")

        ovr = []
        if self.worker.battery_override is not None: ovr.append("B")
        if self.worker.temp_override is not None: ovr.append("T")
        if self.worker.hum_override is not None: ovr.append("H")
        ovr_s = f"  [manual:{''.join(ovr)}]" if ovr else ""
        self.v_batt.set(f"{battery}%{ovr_s if 'B' in ovr else ''}")
        self.v_temp.set(f"{temp}C")
        self.v_hum.set(f"{hum}%")
        self.v_alarm.set(f"Alarm: 0x{alarm:04X} ({sim.alarm_text(alarm)})")
        self.l_alarm.configure(foreground="red" if alarm else "green")

        self.v_at.set(at_cmd)
        self.v_resp.set(resp)

        self.after(250, self._refresh)

    def _drain_events(self):
        try:
            while True:
                kind, msg = self.worker.events.get_nowait()
                if kind == "log":
                    self.txt.configure(state="normal")
                    self.txt.insert("end", msg + "\n")
                    self.txt.see("end")
                    # Cap the log to the last ~500 lines.
                    if int(self.txt.index("end-1c").split(".")[0]) > 500:
                        self.txt.delete("1.0", "100.0")
                    self.txt.configure(state="disabled")
        except queue.Empty:
            pass
        self.after(100, self._drain_events)

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
    p.add_argument("--sn", required=True, help="sensor serial number, hex")
    p.add_argument("--data-id", type=lambda s: int(s, 0), default=1)
    p.add_argument("--firmware-version", type=lambda s: int(s, 0), default=1)
    p.add_argument("--battery", type=int, default=100)
    p.add_argument("--battery-min", type=int, default=sim.DEFAULT_BATTERY_MIN)
    p.add_argument("--default-interval", type=float, default=sim.DEFAULT_INTERVAL_S)
    p.add_argument("--battery-period", type=float, default=sim.BATTERY_PERIOD_S)
    p.add_argument("--gen-period", type=float, default=sim.GEN_PERIOD_S)
    p.add_argument("--packed", action="store_true")
    p.add_argument("--skip-sn-check", action="store_true")
    args = p.parse_args(argv)

    try:
        sn = int(args.sn, 16)
    except ValueError:
        sys.stderr.write(f"error: --sn not valid hex: {args.sn!r}\n")
        return 2
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

    if not args.skip_sn_check:
        if not sim.provision_sn(io, sn):
            io.close()
            return 1

    state = sim.SensorState(
        sn=sn, battery=max(0, min(100, args.battery)),
        fw_version=args.firmware_version,
        sleep_time_sec=int(args.default_interval),
        battery_level_min=args.battery_min,
    )
    worker = SensorWorker(state, io, args)
    worker.start()

    gui = SensorGUI(worker)
    try:
        gui.mainloop()
    finally:
        worker.stop()
        io.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
