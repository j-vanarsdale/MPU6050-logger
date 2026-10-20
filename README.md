# mpu6050-imu

Eventide module for an **MPU-6050** 6-axis IMU on I2C. It samples accel, gyro and
temperature at a fixed rate, estimates roll/pitch/yaw, records UTC-timestamped JSONL logs, and
serves a JSON API that the dashboard widgets use.

## Wiring (Raspberry Pi)

| MPU-6050 | Pi header            |
| -------- | -------------------- |
| VCC      | 3.3 V (pin 1)        |
| GND      | GND (pin 6)          |
| SDA      | GPIO2 / SDA (pin 3)  |
| SCL      | GPIO3 / SCL (pin 5)  |
| AD0      | GND = 0x68, 3.3 V = 0x69 |

Many breakout boards have a 3.3 V regulator and pull-ups; check yours before
feeding it 5 V.

## Install

From the dashboard's MODULES tab: paste the repo URL, or upload a zip of this
folder. Then check the sensor is visible:

```
ls /dev/i2c-1            # if missing: sudo raspi-config nonint do_i2c 0 && reboot
i2cdetect -y 1           # expect 68 (or 69)
```

No sensor attached? The program still installs and runs, reporting
`connected: false` and retrying every 2 s. To exercise the whole pipeline on
a bench with no hardware, add `--simulate` (or `--simulate still`) to the
program command.

## Dashboard components

| Component        | Region  | What it does |
| ---------------- | ------- | ------------ |
| IMU ATTITUDE     | centre  | 3D attitude view (yaw → pan, pitch → tilt, roll → roll) |
| IMU READOUT      | sidebar | live accel / gyro / angles / temperature / actual sample rate |
| IMU SETTINGS     | sidebar | sample rate, accel & gyro range, low-pass filter, filter weight |
| IMU RECORD       | sidebar | start/stop recording; also driven by MASTER RECORD |

## Arguments

Install-time defaults (editable from the MODULES tab; editing restarts the program).

| Argument             | Default     | Notes |
| -------------------- | ----------- | ----- |
| `i2c_bus`            | `1`         | `/dev/i2c-N` |
| `i2c_addr`           | `104`       | decimal; `104` = 0x68, `105` = 0x69 |
| `sample_rate`        | `100`       | Hz, 1–1000 (Python timing degrades above ~400 Hz) |
| `accel_range`        | `4`         | ±g: 2 / 4 / 8 / 16 |
| `gyro_range`         | `500`       | ±°/s: 250 / 500 / 1000 / 2000 |
| `dlpf`               | `3`         | hardware low-pass, 0 (wide) … 6 (5 Hz) |
| `filter_alpha`       | `0.98`      | complementary filter gyro weight |
| `calibrate_on_start` | `1`         | gyro-bias estimate at startup (needs the sensor still; skipped with a log line if it is moving) |
| `autorecord`         | `0`         | `1` = record from program start (after the startup calibration finishes) |
| `segment_seconds`    | `600`       | new file every N seconds while recording; `0` = one file |
| `host`               | `127.0.0.1` | API bind address (the dashboard proxy only needs localhost) |

Settings changed live through the SETTINGS form or `PUT /api/settings` apply
immediately but are **not persisted**; edit the arguments to change the boot defaults.

### Second sensor

The program is copyable on `i2c_addr`. Put the second MPU-6050 on the same bus
with AD0 high and use **ADD COPY** with value `105`. The copy gets its own API
port, widgets and `mpu6050-105` recording directory. (Copies are identified by
address, so two sensors that share an address on different buses can't both be
copies of this program.)

## Recordings

**Nothing is written until recording is started** — press IMU RECORD (or MASTER RECORD),
or set `autorecord` to `1`. Files go to `<recordings_dir>/mpu6050/mpu6050_<UTC start>.jsonl`
and appear in the PLAYBACK tab (which only lists `.jsonl`, `.raw`, `.mp4`, `.h264` and
`.basler` files).

Line 1 of each file is a header (sensor settings, calibration state and biases); every
following line is one sample:

```json
{"type":"header","format":"eventide-imu-jsonl","version":1,"sensor":"MPU-6050","start_utc":"...Z","sample_rate_hz":100,"calibrated":true,"gyro_bias_dps":[0.8,-0.5,0.3],...}
{"t_utc":"2026-10-19T23:51:17.512345Z","t_ns":1792453877512345678,"seq":212,"accel_g":{"x":0.001,"y":0.0,"z":1.003},"gyro_dps":{"x":0.0,"y":-0.02,"z":0.01},"temp_c":25.0,"roll_deg":0.0,"pitch_deg":0.0,"yaw_deg":0.01}
```

Accel/gyro/angles are bias-corrected. `t_utc` / `t_ns` come from the system UTC clock
(as good as the payload RTC/NTP); use `t_ns` to merge with other sensors.

## Calibration

* **Gyro bias** is estimated for ~2 s at startup (`calibrate_on_start`). Keep the sensor
  still; if it moves the attempt is rejected and the program carries on uncalibrated.
  It runs once per program start, not on sensor reconnects. Re-run any time with
  `POST /api/calibrate`.
* **Accel offsets** are not corrected unless you call `/api/calibrate` with
  `{"accel": true}` with the sensor level, Z up.

## API

Reach it through the backend: `/proxy/mpu6050-imu/http_api/<path>`.

| Method | Path             | Body / response |
| ------ | ---------------- | --------------- |
| GET    | `/api/data`      | latest sample: `ax ay az gx gy gz temp_c roll pitch yaw rate_hz sample_count calibrated recording` (`{"ok": false, "error": …}` when the sensor is down) |
| GET    | `/api/status`    | connection, WHO_AM_I, error counts, actual rate, calibration biases |
| GET/PUT| `/api/settings`  | `{sample_rate, accel_range, gyro_range, dlpf, filter_alpha}`. Any subset on PUT; ranges snap to the nearest valid value; the effective settings are returned |
| GET/PUT| `/api/recording` | GET → `{recording, recording_control, current_file}`; PUT `{"recording": bool, "duration_seconds"?: n}` |
| POST   | `/api/calibrate` | `{"seconds"?: 2, "accel"?: false}`. Holds still for N seconds, sets gyro bias. `accel: true` also zeroes accel offsets and requires the sensor **level, Z up**. Returns 409 if it moved |
| POST   | `/api/zero`      | zero the yaw estimate |

```
curl -X POST http://payload/proxy/mpu6050-imu/http_api/api/calibrate -d '{"seconds":3}'
```

## Limits worth knowing

- **Yaw drifts.** The MPU-6050 has no magnetometer, so yaw is integrated gyro
  rate. Good for short-term relative heading; use `/api/zero` to re-reference.
- Roll/pitch use a simple complementary filter (accelerometer correction is
  skipped while |a| is more than 0.3 g from 1 g). It is not a full AHRS, so
  expect errors during sustained high-g manoeuvres.
- Timing is software-paced from Python; the READOUT's RATE row shows the rate
  actually achieved.
- The CSV recorder writes from the sampling thread (flushed once per second).
  On slow storage at high rates, record to fast media.
- A mounting orientation other than "Z up, X forward" just changes which axis
  is called roll/pitch/yaw; there is no mounting-rotation setting yet.
