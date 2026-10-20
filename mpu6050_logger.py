#!/usr/bin/env python3
"""MPU-6050 datalogger for the Eventide module system.

One foreground process, managed by supervisord:

  * a sampler thread that reads the sensor over I2C at a fixed rate, applies
    bias correction and a complementary filter (roll / pitch / yaw estimate),
  * a JSONL recorder with segment rotation (files land in --output-dir, which the
    PLAYBACK tab lists),
  * a small JSON HTTP API (stdlib only) that the dashboard reaches through
    /proxy/<module>/http_api/...

The sensor being absent is not fatal: the sampler keeps retrying, so bench
installs and hot-plugging work. Use --simulate to run without hardware.
"""

import argparse
import json
import math
import os
import signal
import struct
import sys
import threading
import time
import traceback
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

VERSION = "1.0.0"

# --- MPU-6050 registers -----------------------------------------------------
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B  # 14 bytes: accel XYZ, temp, gyro XYZ (big-endian int16)
REG_PWR_MGMT_1 = 0x6B
REG_WHO_AM_I = 0x75

# range -> (register bits, LSB per unit)
ACCEL_RANGES = {2: (0, 16384.0), 4: (1, 8192.0), 8: (2, 4096.0), 16: (3, 2048.0)}
GYRO_RANGES = {250: (0, 131.0), 500: (1, 65.5), 1000: (2, 32.8), 2000: (3, 16.4)}
EXPECTED_WHO_AM_I = {0x68, 0x70, 0x71, 0x72}  # 0x68 genuine; others are common clones



def log(msg):
    print(f"[mpu6050] {msg}", flush=True)


def utc_iso(t_ns):
    """Epoch nanoseconds -> 'YYYY-MM-DDTHH:MM:SS.ffffffZ'."""
    dt = datetime.fromtimestamp(t_ns / 1e9, tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def wrap180(angle):
    return (angle + 180.0) % 360.0 - 180.0


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def snap(value, options):
    return min(options, key=lambda o: abs(o - value))


def normalize_settings(raw, base):
    """Merge `raw` into `base`, validating and snapping. Raises ValueError."""
    out = dict(base)
    try:
        if "sample_rate" in raw:
            out["sample_rate"] = int(clamp(round(float(raw["sample_rate"])), 1, 1000))
        if "accel_range" in raw:
            out["accel_range"] = snap(float(raw["accel_range"]), list(ACCEL_RANGES))
        if "gyro_range" in raw:
            out["gyro_range"] = snap(float(raw["gyro_range"]), list(GYRO_RANGES))
        if "dlpf" in raw:
            out["dlpf"] = int(clamp(round(float(raw["dlpf"])), 0, 6))
        if "filter_alpha" in raw:
            out["filter_alpha"] = float(clamp(float(raw["filter_alpha"]), 0.0, 1.0))
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"invalid setting value: {exc}")
    return out


# --- Simulated bus (for --simulate) ----------------------------------------
class SimBus:
    """Emulates just enough of an MPU-6050 for bench testing.

    mode "motion": gentle rocking about roll/pitch plus a yaw rate.
    mode "still":  stationary on a table (gravity only) - lets calibration pass.
    A constant gyro bias is always present so calibration is observable.
    """

    GYRO_BIAS = (0.8, -0.5, 0.3)  # deg/s

    def __init__(self, mode="motion"):
        self.mode = mode
        self.regs = {}
        self.t0 = time.monotonic()

    def write_byte_data(self, addr, reg, value):
        self.regs[reg] = value

    def read_byte_data(self, addr, reg):
        return 0x68 if reg == REG_WHO_AM_I else self.regs.get(reg, 0)

    def read_i2c_block_data(self, addr, reg, length):
        import random
        t = time.monotonic() - self.t0
        if self.mode == "motion":
            roll = 20.0 * math.sin(2 * math.pi * 0.2 * t)
            pitch = 10.0 * math.sin(2 * math.pi * 0.13 * t)
            gx = 20.0 * 2 * math.pi * 0.2 * math.cos(2 * math.pi * 0.2 * t)
            gy = 10.0 * 2 * math.pi * 0.13 * math.cos(2 * math.pi * 0.13 * t)
            gz = 5.0 * math.sin(2 * math.pi * 0.05 * t)
        else:
            roll = pitch = gx = gy = gz = 0.0
        r, p = math.radians(roll), math.radians(pitch)
        ax, ay, az = -math.sin(p), math.sin(r) * math.cos(p), math.cos(r) * math.cos(p)
        a_lsb = [16384.0, 8192.0, 4096.0, 2048.0][(self.regs.get(REG_ACCEL_CONFIG, 0) >> 3) & 3]
        g_lsb = [131.0, 65.5, 32.8, 16.4][(self.regs.get(REG_GYRO_CONFIG, 0) >> 3) & 3]
        n = lambda s: random.gauss(0, s)
        vals = [
            ax * a_lsb + n(20), ay * a_lsb + n(20), az * a_lsb + n(20),
            (25.0 - 36.53) * 340.0,
            (gx + self.GYRO_BIAS[0]) * g_lsb + n(3),
            (gy + self.GYRO_BIAS[1]) * g_lsb + n(3),
            (gz + self.GYRO_BIAS[2]) * g_lsb + n(3),
        ]
        vals = [int(clamp(round(v), -32768, 32767)) for v in vals]
        return list(struct.pack(">7h", *vals))[:length]

    def close(self):
        pass


# --- JSONL recorder ---------------------------------------------------------
class Recorder:
    def __init__(self, out_dir, segment_seconds):
        self.out_dir = out_dir
        self.segment_seconds = segment_seconds
        self._lock = threading.Lock()
        self._fh = None
        self.header_fn = None  # set in main(): returns the header dict for a new file
        self._seg_start = 0.0
        self._last_flush = 0.0
        self._stop_at = None
        self.active = False
        self.current_file = None
        self.last_error = None

    def _open_segment(self):
        os.makedirs(self.out_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        name = f"mpu6050_{stamp}.jsonl"
        path = os.path.join(self.out_dir, name)
        n = 1
        while os.path.exists(path):  # stop/start inside the same second
            name = f"mpu6050_{stamp}_{n}.jsonl"
            path = os.path.join(self.out_dir, name)
            n += 1
        self._fh = open(path, "w", encoding="utf-8")
        if self.header_fn:
            self._fh.write(json.dumps(self.header_fn(), separators=(",", ":")) + "\n")
        self._seg_start = self._last_flush = time.monotonic()
        self.current_file = name
        log(f"recording -> {path}")

    def _close_locked(self):
        if self._fh:
            try:
                self._fh.flush()
                self._fh.close()
            except OSError as exc:
                log(f"error closing recording file: {exc}")
        self._fh = None
        self.active = False
        self._stop_at = None
        self.current_file = None

    def start(self, duration=None):
        with self._lock:
            if self.active:
                return
            self.last_error = None
            self._open_segment()
            self.active = True
            self._stop_at = time.monotonic() + duration if duration else None

    def stop(self):
        with self._lock:
            if self.active:
                log("recording stopped")
            self._close_locked()

    def write(self, row):
        with self._lock:
            if not self.active:
                return
            now = time.monotonic()
            try:
                if self._stop_at is not None and now >= self._stop_at:
                    log("recording duration reached")
                    self._close_locked()
                    return
                if self.segment_seconds and now - self._seg_start >= self.segment_seconds:
                    self._fh.close()
                    self._open_segment()
                self._fh.write(json.dumps(row, separators=(",", ":")) + "\n")
                if now - self._last_flush >= 1.0:
                    self._fh.flush()
                    self._last_flush = now
            except OSError as exc:  # disk full, card removed, ...
                self.last_error = str(exc)
                log(f"recording failed, stopping: {exc}")
                self._close_locked()

    def state(self):
        return {"recording": self.active, "recording_control": True,
                "current_file": self.current_file, "error": self.last_error}


# --- Sampler ----------------------------------------------------------------
class Sampler(threading.Thread):
    def __init__(self, bus_factory, addr, settings, recorder, calibrate_on_start):
        super().__init__(daemon=True, name="sampler")
        self.bus_factory = bus_factory
        self.addr = addr
        self.settings = dict(settings)
        self.recorder = recorder
        self.calibrate_on_start = bool(calibrate_on_start)
        self.autorecord = False          # set by main(); recording begins once the sensor is ready
        self._autorecord_done = False

        self._halt = threading.Event()
        self._lock = threading.Lock()
        self._pending_settings = None
        self._cal = None
        self._startup_cal_done = False
        self._zero_yaw = False

        self.gyro_bias = [0.0, 0.0, 0.0]   # deg/s
        self.accel_bias = [0.0, 0.0, 0.0]  # g
        self.calibrated = False

        self._att_init = False
        self.roll = self.pitch = self.yaw = 0.0

        self.latest = {"ok": False, "error": "no sample yet"}
        self.connected = False
        self.who_am_i = None
        self.last_error = None
        self.error_count = 0
        self.sample_count = 0
        self.rate_hz = 0.0
        self.started = time.monotonic()
        self._last_err_log = 0.0

    # ---- public (called from HTTP threads) ----
    def stop(self):
        self._halt.set()

    def update_settings(self, raw):
        with self._lock:
            base = self._pending_settings or self.settings
            new = normalize_settings(raw, base)
            self._pending_settings = new
        return new

    def zero_yaw(self):
        self._zero_yaw = True

    def request_calibration(self, seconds, use_accel):
        if not self.connected:
            return 503, {"ok": False, "error": "sensor not connected"}
        cal = {"seconds": seconds, "accel": use_accel, "event": threading.Event(),
               "samples": [], "end": None, "result": None, "wait": True}
        with self._lock:
            if self._cal is not None:
                return 409, {"ok": False, "error": "calibration already running"}
            self._cal = cal
        cal["event"].wait(seconds + 5)
        with self._lock:
            if self._cal is cal:
                self._cal = None
        res = cal["result"] or {"ok": False, "error": "calibration timed out"}
        return (200 if res.get("ok") else 409), res

    def recording_header(self):
        """First line of every recording file (called under the recorder lock)."""
        s = self.settings
        t_ns = time.time_ns()
        return {
            "type": "header", "format": "eventide-imu-jsonl", "version": 1,
            "sensor": "MPU-6050",
            "start_utc": utc_iso(t_ns), "start_ns": t_ns,
            "timebase": "t_ns / t_utc = system UTC clock (CLOCK_REALTIME) when the sample was processed",
            "sample_rate_hz": s["sample_rate"], "accel_range_g": s["accel_range"],
            "gyro_range_dps": s["gyro_range"], "dlpf": s["dlpf"],
            "filter_alpha": s["filter_alpha"],
            "units": {"accel_g": "g", "gyro_dps": "deg/s", "temp_c": "C", "angles": "deg"},
            "corrections": "accel/gyro are bias-corrected using the values below",
            "gyro_bias_dps": [round(v, 4) for v in self.gyro_bias],
            "accel_bias_g": [round(v, 5) for v in self.accel_bias],
            "calibrated": self.calibrated,
            "yaw_note": "yaw is integrated gyro only (no magnetometer) and drifts",
        }

    def status(self):
        return {
            "version": VERSION, "connected": self.connected, "address": self.addr,
            "who_am_i": None if self.who_am_i is None else f"0x{self.who_am_i:02X}",
            "error": self.last_error,
            "error_count": self.error_count, "sample_count": self.sample_count,
            "rate_hz": round(self.rate_hz, 2), "calibrated": self.calibrated,
            "gyro_bias_dps": [round(v, 4) for v in self.gyro_bias],
            "accel_bias_g": [round(v, 5) for v in self.accel_bias],
            "uptime_s": round(time.monotonic() - self.started, 1),
        }

    # ---- thread body ----
    def run(self):
        while not self._halt.is_set():
            bus = None
            try:
                bus = self.bus_factory()
                self._init_device(bus)
                self.connected = True
                self.last_error = None
                self._sample_loop(bus)
            except OSError as exc:
                self.connected = False
                self.last_error = str(exc)
                self.latest = {"ok": False, "error": self.last_error}
                now = time.monotonic()
                if now - self._last_err_log > 30:
                    log(f"sensor unavailable ({exc}); retrying every 2 s")
                    self._last_err_log = now
            except Exception:  # a bug must not silently kill the sampler
                self.connected = False
                self.last_error = "internal error (see log)"
                log("unexpected error:\n" + traceback.format_exc())
            finally:
                self.connected = False
                if bus is not None:
                    try:
                        bus.close()
                    except Exception:
                        pass
            self._halt.wait(2.0)

    def _init_device(self, bus):
        self.who_am_i = bus.read_byte_data(self.addr, REG_WHO_AM_I)
        if self.who_am_i not in EXPECTED_WHO_AM_I:
            log(f"WARNING: WHO_AM_I=0x{self.who_am_i:02X} at 0x{self.addr:02X} "
                f"(expected 0x68) - continuing anyway")
        bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0x80)  # device reset
        time.sleep(0.1)
        bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0x01)  # wake, PLL on gyro X
        time.sleep(0.05)
        self._apply_regs(bus)
        self._att_init = False
        log(f"connected: addr 0x{self.addr:02X}, WHO_AM_I=0x{self.who_am_i:02X}")
        if self.calibrate_on_start and not self._startup_cal_done:
            self._startup_cal_done = True
            with self._lock:
                self._cal = {"seconds": 2.0, "accel": False, "event": threading.Event(),
                             "samples": [], "end": None, "result": None, "wait": False,
                             "startup": True}
            log("startup calibration: keep the sensor still for ~2 s")
        else:
            self._maybe_autorecord()

    def _maybe_autorecord(self):
        """--autorecord: begin recording once, after the sensor is up (and calibrated)."""
        if self.autorecord and not self._autorecord_done:
            self._autorecord_done = True
            self.recorder.start()

    def _apply_regs(self, bus):
        s = self.settings
        gyro_out = 8000.0 if s["dlpf"] in (0, 7) else 1000.0
        div = int(clamp(round(gyro_out / s["sample_rate"]) - 1, 0, 255))
        bus.write_byte_data(self.addr, REG_CONFIG, s["dlpf"])
        bus.write_byte_data(self.addr, REG_GYRO_CONFIG, GYRO_RANGES[s["gyro_range"]][0] << 3)
        bus.write_byte_data(self.addr, REG_ACCEL_CONFIG, ACCEL_RANGES[s["accel_range"]][0] << 3)
        bus.write_byte_data(self.addr, REG_SMPLRT_DIV, div)

    def _sample_loop(self, bus):
        period = 1.0 / self.settings["sample_rate"]
        next_t = time.perf_counter()
        last_t = None
        consecutive_errors = 0
        win_start, win_n = time.perf_counter(), 0

        while not self._halt.is_set():
            with self._lock:
                pending, self._pending_settings = self._pending_settings, None
            if pending:
                self.settings = pending
                self._apply_regs(bus)
                period = 1.0 / self.settings["sample_rate"]
                next_t = time.perf_counter()
                log(f"settings applied: {self.settings}")

            try:
                raw = bus.read_i2c_block_data(self.addr, REG_ACCEL_XOUT_H, 14)
                consecutive_errors = 0
            except OSError as exc:
                consecutive_errors += 1
                self.error_count += 1
                self.last_error = str(exc)
                if consecutive_errors >= 10:
                    raise
                self._halt.wait(period)
                continue

            t = time.perf_counter()
            self._process(raw, t, None if last_t is None else t - last_t)
            last_t = t

            win_n += 1
            if t - win_start >= 1.0:
                self.rate_hz = win_n / (t - win_start)
                win_start, win_n = t, 0

            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                self._halt.wait(delay)
            elif delay < -5 * period:
                next_t = time.perf_counter()  # fell behind; resync instead of bursting

    def _process(self, raw, t, dt):
        s = self.settings
        v = struct.unpack(">7h", bytes(raw))
        a_lsb = ACCEL_RANGES[s["accel_range"]][1]
        g_lsb = GYRO_RANGES[s["gyro_range"]][1]
        a_raw = [v[0] / a_lsb, v[1] / a_lsb, v[2] / a_lsb]
        g_raw = [v[4] / g_lsb, v[5] / g_lsb, v[6] / g_lsb]
        temp = v[3] / 340.0 + 36.53

        self._feed_calibration(a_raw, g_raw, t)

        ax, ay, az = (a_raw[i] - self.accel_bias[i] for i in range(3))
        gx, gy, gz = (g_raw[i] - self.gyro_bias[i] for i in range(3))

        if self._zero_yaw:
            self.yaw = 0.0
            self._zero_yaw = False

        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        if not self._att_init:
            self.roll, self.pitch, self._att_init = roll_acc, pitch_acc, True
        else:
            dt = clamp(dt or 0.0, 0.0, 0.5)
            norm = math.sqrt(ax * ax + ay * ay + az * az)
            # Only trust the accelerometer when it is close to 1 g (not accelerating hard)
            alpha = s["filter_alpha"] if abs(norm - 1.0) < 0.3 else 1.0
            self.roll = wrap180(self.roll + gx * dt)
            self.pitch = wrap180(self.pitch + gy * dt)
            self.roll = wrap180(self.roll + (1 - alpha) * wrap180(roll_acc - self.roll))
            self.pitch = wrap180(self.pitch + (1 - alpha) * wrap180(pitch_acc - self.pitch))
            self.yaw = wrap180(self.yaw + gz * dt)  # gyro-only: drifts, no magnetometer

        self.sample_count += 1
        t_ns = time.time_ns()
        self.latest = {
            "ok": True, "timestamp": round(t_ns / 1e9, 6), "t_utc": utc_iso(t_ns),
            "ax": round(ax, 5), "ay": round(ay, 5), "az": round(az, 5),
            "gx": round(gx, 4), "gy": round(gy, 4), "gz": round(gz, 4),
            "temp_c": round(temp, 2),
            "roll": round(self.roll, 3), "pitch": round(self.pitch, 3), "yaw": round(self.yaw, 3),
            "rate_hz": round(self.rate_hz, 2), "sample_count": self.sample_count,
            "calibrated": self.calibrated, "recording": self.recorder.active,
        }
        if self.recorder.active:
            self.recorder.write({
                "t_utc": utc_iso(t_ns), "t_ns": t_ns, "seq": self.sample_count,
                "accel_g": {"x": round(ax, 5), "y": round(ay, 5), "z": round(az, 5)},
                "gyro_dps": {"x": round(gx, 4), "y": round(gy, 4), "z": round(gz, 4)},
                "temp_c": round(temp, 2),
                "roll_deg": round(self.roll, 3), "pitch_deg": round(self.pitch, 3),
                "yaw_deg": round(self.yaw, 3),
            })

    # ---- calibration ----
    def _feed_calibration(self, a_raw, g_raw, t):
        cal = self._cal
        if cal is None:
            return
        if cal["end"] is None:
            cal["end"] = t + cal["seconds"]
        cal["samples"].append(a_raw + g_raw)
        if t >= cal["end"]:
            cal["result"] = self._finish_calibration(cal)
            if cal.get("startup"):
                self._maybe_autorecord()   # success or not: don't wait forever
            with self._lock:
                if self._cal is cal:
                    self._cal = None
            cal["event"].set()

    def _finish_calibration(self, cal):
        data = cal["samples"]
        n = len(data)
        if n < 10:
            return {"ok": False, "error": f"too few samples ({n}); raise the sample rate"}
        mean = [sum(row[i] for row in data) / n for i in range(6)]
        g_dev = max(abs(row[i] - mean[i]) for row in data for i in range(3, 6))
        if g_dev > 5.0:
            res = {"ok": False, "error": f"sensor was moving (gyro deviation {g_dev:.1f} deg/s); "
                                         "keep it still and retry"}
            log(f"calibration failed: {res['error']}")
            return res
        accel_bias = None
        if cal["accel"]:
            if abs(mean[0]) > 0.2 or abs(mean[1]) > 0.2 or mean[2] < 0.8:
                res = {"ok": False, "error": "accel calibration needs the sensor level, Z up"}
                log(f"calibration failed: {res['error']}")
                return res
            accel_bias = [mean[0], mean[1], mean[2] - 1.0]
            self.accel_bias = accel_bias
        self.gyro_bias = mean[3:6]
        self.calibrated = True
        self._att_init = False   # re-seed attitude from the corrected accel
        self.yaw = 0.0
        log(f"calibrated: gyro bias {[round(x, 3) for x in self.gyro_bias]} deg/s"
            + (f", accel bias {[round(x, 4) for x in accel_bias]} g" if accel_bias else ""))
        return {"ok": True, "samples": n,
                "gyro_bias_dps": [round(x, 4) for x in self.gyro_bias],
                "accel_bias_g": None if accel_bias is None else [round(x, 5) for x in accel_bias]}


# --- HTTP API ---------------------------------------------------------------
def make_handler(sampler, recorder):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "mpu6050-logger/" + VERSION

        def log_message(self, fmt, *args):  # keep supervisor logs quiet
            pass

        def _send(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json_body(self):
            length = int(self.headers.get("Content-Length") or 0)
            if length > 65536:
                raise ValueError("request body too large")
            raw = self.rfile.read(length) if length else b""
            if not raw:
                return {}
            try:
                body = json.loads(raw)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSON: {exc}")
            if not isinstance(body, dict):
                raise ValueError("JSON body must be an object")
            return body

        def _dispatch(self, method):
            path = urlparse(self.path).path.rstrip("/") or "/"
            try:
                handler = ROUTES.get((method, path))
                if handler is None:
                    return self._send(404, {"error": f"no route {method} {path}"})
                code, obj = handler(self)
                self._send(code, obj)
            except ValueError as exc:
                self._send(400, {"error": str(exc)})
            except Exception:
                log("handler error:\n" + traceback.format_exc())
                self._send(500, {"error": "internal error"})

        def do_GET(self):
            self._dispatch("GET")

        def do_PUT(self):
            self._dispatch("PUT")

        def do_POST(self):
            self._dispatch("POST")

    # -- routes --
    def get_data(h):
        return 200, sampler.latest

    def get_status(h):
        return 200, sampler.status()

    def get_settings(h):
        return 200, sampler.settings

    def put_settings(h):
        body = h._json_body()
        return 200, sampler.update_settings(body)

    def get_recording(h):
        return 200, recorder.state()

    def put_recording(h):
        body = h._json_body()
        if "recording" not in body or not isinstance(body["recording"], (bool, int)):
            raise ValueError('body must contain boolean "recording"')
        if body["recording"]:
            duration = body.get("duration_seconds")
            if duration is not None:
                duration = float(duration)
                if duration <= 0:
                    duration = None
            recorder.start(duration)
        else:
            recorder.stop()
        return 200, recorder.state()

    def post_calibrate(h):
        body = h._json_body()
        seconds = clamp(float(body.get("seconds", 2.0)), 0.5, 30.0)
        return sampler.request_calibration(seconds, bool(body.get("accel", False)))

    def post_zero(h):
        sampler.zero_yaw()
        return 200, {"ok": True}

    ROUTES = {
        ("GET", "/api/data"): get_data,
        ("GET", "/api/status"): get_status,
        ("GET", "/api/settings"): get_settings,
        ("PUT", "/api/settings"): put_settings,
        ("POST", "/api/settings"): put_settings,
        ("GET", "/api/recording"): get_recording,
        ("PUT", "/api/recording"): put_recording,
        ("POST", "/api/recording"): put_recording,
        ("POST", "/api/calibrate"): post_calibrate,
        ("POST", "/api/zero"): post_zero,
    }
    return Handler


# --- main -------------------------------------------------------------------
def int_auto(text):
    return int(text, 0)  # accepts 104 and 0x68


def main():
    p = argparse.ArgumentParser(description="MPU-6050 datalogger")
    p.add_argument("--bus", type=int, default=1)
    p.add_argument("--addr", type=int_auto, default=0x68)
    p.add_argument("--rate", type=int, default=100)
    p.add_argument("--accel-range", type=int, default=4)
    p.add_argument("--gyro-range", type=int, default=500)
    p.add_argument("--dlpf", type=int, default=3)
    p.add_argument("--alpha", type=float, default=0.98)
    p.add_argument("--calibrate-on-start", type=int, default=1)
    p.add_argument("--autorecord", type=int, default=0)
    p.add_argument("--segment-seconds", type=int, default=600)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--simulate", nargs="?", const="motion", choices=["motion", "still"],
                   help="use a simulated sensor instead of I2C hardware")
    args = p.parse_args()

    settings = normalize_settings(
        {"sample_rate": args.rate, "accel_range": args.accel_range,
         "gyro_range": args.gyro_range, "dlpf": args.dlpf, "filter_alpha": args.alpha},
        {})

    if args.simulate:
        log(f"SIMULATION MODE ({args.simulate}) - no hardware is being read")
        bus_factory = lambda: SimBus(args.simulate)
    else:
        try:
            from smbus2 import SMBus
        except ImportError:
            log("ERROR: smbus2 is not installed in this Python environment "
                "(run via the module venv, or use --simulate)")
            return 2
        bus_factory = lambda: SMBus(args.bus)

    recorder = Recorder(args.output_dir, args.segment_seconds)
    sampler = Sampler(bus_factory, args.addr, settings, recorder, args.calibrate_on_start)
    recorder.header_fn = sampler.recording_header
    sampler.autorecord = bool(args.autorecord)
    sampler.start()

    server = ThreadingHTTPServer((args.host, args.port), make_handler(sampler, recorder))
    server.daemon_threads = True

    def shutdown(signum, frame):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    log(f"API on http://{args.host}:{args.port}  bus={args.bus} addr=0x{args.addr:02X} "
        f"rate={settings['sample_rate']} Hz  output={args.output_dir}")
    try:
        server.serve_forever()
    finally:
        sampler.stop()
        recorder.stop()  # flush the recording so a supervisor stop never loses data
        log("stopped")


if __name__ == "__main__":
    sys.exit(main())
