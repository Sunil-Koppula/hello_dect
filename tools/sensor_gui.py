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
        # Serializes everything we transmit on the shared serial port: an
        # AT#REPORT send (report_loop) and a large-data transfer (_ld_worker)
        # must never overlap. Writing an AT#REPORT into the middle of the
        # flow-controlled AT#LD stream breaks the one-line-in-flight contract
        # (wedges the firmware RX) and its OK gets mis-consumed as a chunk ack.
        # Force-alarm / stop-alarm / interval / manual sends all go through
        # _send, so this lock covers the whole "report vs large-data" overlap.
        self.tx_lock = threading.Lock()

        # Manual overrides set from the GUI. When *_override is not None the
        # report loop uses it instead of the random/drained value, and reading
        # regeneration is suppressed while temp/hum are overridden.
        self.battery_override = None
        self.temp_override = None
        self.hum_override = None
        self.force_alarm = 0            # extra alarm bits OR-ed in
        self.stop_alarm = False         # when True, alarm flags are forced to 0
        self._send_now = threading.Event()
        # data_id auto-increments per report, wrapping at DATA_ID_MAX -> 1.
        self.data_id = max(1, min(sim.DATA_ID_MAX, args.data_id))
        # Separate id stream + busy guard for large-data (AT#LD) transfers.
        self.ld_data_id = 1
        self._ld_busy = False

    # -- helpers ---------------------------------------------------------
    def log(self, msg: str) -> None:
        self.events.put(("log", f"[{time.strftime('%H:%M:%S')}] {msg}"))

    def request_send(self) -> None:
        self._send_now.set()

    def current_alarm(self) -> int:
        """Effective alarm bitmask. 'Stop alarm' forces 0 regardless of config
        thresholds or a forced alarm; otherwise it's the computed flags OR the
        manually forced bits."""
        if self.stop_alarm:
            return 0
        return self.state.compute_alarm_flags() | self.force_alarm

    # -- large-data (AT#LDINIT + AT#LD) ----------------------------------
    def send_large_data(self, total_bytes: int) -> None:
        """Kick off a dummy large-data transfer on a background thread so the Tk
        loop stays responsive. total_bytes is the WHOLE sensor_large_data_
        structure_t size (header + report_info_3300 + sound_record), so "100 KB"
        is 100 KB total. Ignored if one is in flight."""
        if self._ld_busy:
            self.log("LD: a transfer is already in progress")
            return
        self._ld_busy = True
        threading.Thread(target=self._ld_worker, args=(total_bytes,),
                         daemon=True).start()

    def _ld_worker(self, total_bytes: int) -> None:
        data_id = self.ld_data_id
        try:
            # The transfer payload is a serialized sensor_large_data_structure_t
            # sized so the WHOLE blob (header + report_info_3300 + sound_record)
            # is exactly total_bytes — the sound_record is sized to fill it.
            sound_len = sim.ld_sound_record_len_for_total(total_bytes)
            sound_record = sim.make_dummy_sound_record(sound_len)
            blob = sim.build_large_data_blob(self.state, sound_record)
            assert len(blob) == total_bytes
            chunks = list(sim.iter_ld_chunks(blob))
            n = len(chunks)
            last = len(chunks[-1][2])
            data_crc32 = zlib.crc32(blob) & 0xFFFFFFFF

            # Own the serial link for the WHOLE transfer. While we hold tx_lock,
            # report_loop's _send blocks, so no AT#REPORT (force-alarm / stop /
            # interval / manual) can be injected into this flow-controlled
            # AT#LD stream and no report OK can steal a chunk ack from resp_q.
            with self.tx_lock:
                sim._drain_resp_q(self.io)

                # Announce, then wait for the device to consume it.
                init = sim.build_at_ldinit(self.state.sn, data_id, len(blob), n,
                                           last, data_crc32)
                self.io.write(init.encode("ascii", errors="replace"))
                self.log(f">>> AT#LDINIT id={data_id} total={len(blob)} "
                         f"chunks={n} last={last}")
                sim._await_ack(self.io, sim.LD_ACK_TIMEOUT_S)

                # Flow-controlled: one line in flight, wait for OK/ERROR per chunk.
                for i, (page, chunk, payload) in enumerate(chunks):
                    if self.stop_evt.is_set():
                        self.log("LD: aborted (shutting down)")
                        return
                    line = sim.build_at_ld(self.state.sn, data_id, page, chunk,
                                           payload)
                    self.io.write(line.encode("ascii", errors="replace"))
                    ack = sim._await_ack(self.io, sim.LD_ACK_TIMEOUT_S)
                    if (i + 1) % 50 == 0 or (i + 1) == n:
                        self.log(f">>> AT#LD {i + 1}/{n} (p{page:02X} c{chunk:02X}) "
                                 f"[{ack or 'timeout'}]")

                self.log(f">>> LD done: {len(blob)} B in {n} chunks (id={data_id})")
                self.ld_data_id = sim.next_data_id(self.ld_data_id)
        except Exception as e:
            self.log(f"LD error: {e!r}")
        finally:
            self._ld_busy = False

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
        alarm = self.current_alarm()
        priority = sim.PRIO_ALARM if alarm else sim.PRIO_NORMAL
        payload = st.snapshot_payload(alarm)
        data_id = self.data_id
        line = sim.build_at_report(st.sn, data_id, payload, priority)
        try:
            # Block if a large-data transfer owns the link, so this report is
            # never written into the middle of its AT#LD stream. The send waits
            # until the transfer completes rather than corrupting it.
            with self.tx_lock:
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

            # While a large-data transfer owns the serial link, transmit NOTHING
            # else. Interleaving an AT#REPORT into the flow-controlled AT#LD
            # stream overruns the firmware's AT RX buffer ("Rx buffer doesn't
            # have enough space"). _ld_busy is set before the LD worker thread
            # starts, so this also closes the gap before it takes tx_lock. Hold
            # the cadence timers at "now" so reports resume cleanly afterwards
            # instead of firing a backlog all at once.
            if self._ld_busy:
                next_battery = max(next_battery, now)
                next_check = max(next_check, now)
                next_report = max(next_report, now)
                self.stop_evt.wait(0.2)
                continue

            if self.battery_override is None and now >= next_battery:
                self.state.drain_battery()
                next_battery += self.args.battery_period

            # Every gen_period: regenerate readings (unless overridden), then
            # CHECK PARAMETERS. If any alarm condition holds, send immediately.
            if now >= next_check:
                if self.temp_override is None and self.hum_override is None:
                    self.state.regen_readings()
                next_check += self.args.gen_period
                alarm = self.current_alarm()
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
                if not line:
                    continue
                # Surface terminal responses (OK/ERROR/#...) so the flow-
                # controlled AT#LD sender can pace on them.
                if line in ("OK", "ERROR") or line.startswith("#"):
                    try:
                        self.io.resp_q.put_nowait(line)
                    except queue.Full:
                        pass
                    continue
                if not line.startswith("AT#CONFIG="):
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
                decoded = sim.decode_config_payload(parsed["payload"],
                                                    self.state.last_report_type)
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
                      ("report_time", "bat_min", "tmax", "tmin", "hmax", "hmin",
                       "tmax2", "tmin2", "hmax2", "hmin2", "us_max", "vib_max",
                       "flags", "ts")}
        rows = [
            ("Report time:", "report_time", "Min Battery Level:", "bat_min"),
            ("Temp1 Max:", "tmax", "Hum1 Max:", "hmax"),
            ("Temp1 Min:", "tmin", "Hum1 Min:", "hmin"),
            ("Temp2 Max:", "tmax2", "Hum2 Max:", "hmax2"),
            ("Temp2 Min:", "tmin2", "Hum2 Min:", "hmin2"),
            ("Ultrasound lvl max:", "us_max", "Vibration lvl max:", "vib_max"),
        ]
        for r, (l1, k1, l2, k2) in enumerate(rows):
            ttk.Label(f, text=l1, width=18).grid(row=r, column=0, sticky="w")
            ttk.Label(f, textvariable=self.v_cfg[k1], width=12).grid(row=r, column=1, sticky="w")
            ttk.Label(f, text=l2, width=18).grid(row=r, column=2, sticky="w")
            ttk.Label(f, textvariable=self.v_cfg[k2], width=12).grid(row=r, column=3, sticky="w")
        last = len(rows)
        ttk.Label(f, text="Config Flags:", width=18).grid(row=last, column=0, sticky="w")
        ttk.Label(f, textvariable=self.v_cfg["flags"]).grid(row=last, column=1, columnspan=3, sticky="w")
        ttk.Label(f, textvariable=self.v_cfg["ts"], foreground="gray").grid(row=last + 1, column=0, columnspan=4, sticky="w")

    def _build_params_panel(self):
        f = self._section("Current Parameters")
        self.v_batt = tk.StringVar()
        self.v_temp = tk.StringVar()
        self.v_hum = tk.StringVar()
        self.v_temp2 = tk.StringVar()
        self.v_hum2 = tk.StringVar()
        self.v_us = tk.StringVar()
        self.v_vib = tk.StringVar()
        self.v_alarm = tk.StringVar()
        ttk.Label(f, text="Battery:", width=12).grid(row=0, column=0, sticky="w")
        ttk.Label(f, textvariable=self.v_batt, width=14).grid(row=0, column=1, sticky="w")
        ttk.Label(f, text="Temp1:", width=10).grid(row=0, column=2, sticky="w")
        ttk.Label(f, textvariable=self.v_temp, width=12).grid(row=0, column=3, sticky="w")
        ttk.Label(f, text="Hum1:", width=8).grid(row=0, column=4, sticky="w")
        ttk.Label(f, textvariable=self.v_hum, width=12).grid(row=0, column=5, sticky="w")
        ttk.Label(f, text="Temp2:", width=10).grid(row=1, column=2, sticky="w")
        ttk.Label(f, textvariable=self.v_temp2, width=12).grid(row=1, column=3, sticky="w")
        ttk.Label(f, text="Hum2:", width=8).grid(row=1, column=4, sticky="w")
        ttk.Label(f, textvariable=self.v_hum2, width=12).grid(row=1, column=5, sticky="w")
        ttk.Label(f, text="Ultrasound:", width=12).grid(row=2, column=0, sticky="w")
        ttk.Label(f, textvariable=self.v_us, width=20).grid(row=2, column=1, columnspan=2, sticky="w")
        ttk.Label(f, text="Vibration:", width=10).grid(row=2, column=3, sticky="w")
        ttk.Label(f, textvariable=self.v_vib, width=20).grid(row=2, column=4, columnspan=2, sticky="w")
        self.l_alarm = ttk.Label(f, textvariable=self.v_alarm, font=("Consolas", 10, "bold"))
        self.l_alarm.grid(row=3, column=0, columnspan=6, sticky="w", pady=(6, 0))

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
        self.v_stop = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="Stop alarm (force flags=0)", variable=self.v_stop,
                        command=self._toggle_stop).grid(row=3, column=0, columnspan=3, sticky="w", pady=4)
        ttk.Button(f, text="Send report now", command=self.worker.request_send).grid(row=2, column=4, columnspan=2, sticky="e")

        # Large-data (sound record) test transfers — own row, below Stop alarm.
        ttk.Label(f, text="Large data:").grid(row=4, column=0, sticky="w", pady=4)
        # Sizes are the WHOLE structure (header + report_info_3300 + sound_record);
        # 200 KB is the max (SENSOR_LARGE_DATA_INFO_MAX).
        ttk.Button(f, text="Send LD 100KB",
                   command=lambda: self.worker.send_large_data(100 * 1024)
                   ).grid(row=4, column=1, columnspan=2, padx=4, sticky="w")
        ttk.Button(f, text="Send LD 200KB",
                   command=lambda: self.worker.send_large_data(200 * 1024)
                   ).grid(row=4, column=3, columnspan=2, padx=4, sticky="w")

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

    def _toggle_stop(self):
        self.worker.stop_alarm = self.v_stop.get()
        self.worker.log(f"stop alarm = {'on (flags forced to 0)' if self.v_stop.get() else 'off'}")

    # -- periodic refresh ------------------------------------------------
    def _refresh(self):
        st = self.state_
        with st.lock:
            sn, rtype = st.sn, st.last_report_type
            sleep_s, bat_min = st.sleep_time_sec, st.battery_level_min
            tmax, tmin, hmax, hmin = st.temp_max, st.temp_min, st.hum_max, st.hum_min
            tmax2, tmin2, hmax2, hmin2 = st.temp_max2, st.temp_min2, st.hum_max2, st.hum_min2
            us_max, vib_max = st.ultrasound_level_max, st.vibration_level_max
            cfg_flags, cfg_ts = st.config_flags, st.last_config_ts
            battery, temp, hum = st.battery, st.temperature, st.humidity
            temp2, hum2 = st.temperature2, st.humidity2
            us_lvl, us_freq = st.ultrasound_level, st.ultrasound_frequency
            vib_lvl, vib_freq = st.vibration_level, st.vibration_frequency
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
        self.v_cfg["tmax2"].set(dash(tmax2, "C"))
        self.v_cfg["tmin2"].set(dash(tmin2, "C"))
        self.v_cfg["hmax2"].set(dash(hmax2, "%"))
        self.v_cfg["hmin2"].set(dash(hmin2, "%"))
        self.v_cfg["us_max"].set(dash(us_max))
        self.v_cfg["vib_max"].set(dash(vib_max))
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
        self.v_temp2.set(f"{temp2}C")
        self.v_hum2.set(f"{hum2}%")
        self.v_us.set(f"lvl={us_lvl} freq={us_freq}")
        self.v_vib.set(f"lvl={vib_lvl} freq={vib_freq}")
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
    p.add_argument("--packed", action="store_true",
                   help="deprecated/no-op: the wire layout is now fixed (packed)")
    p.add_argument("--skip-sn-check", action="store_true")
    args = p.parse_args(argv)

    try:
        sn = int(args.sn, 16)
    except ValueError:
        sys.stderr.write(f"error: --sn not valid hex: {args.sn!r}\n")
        return 2

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
