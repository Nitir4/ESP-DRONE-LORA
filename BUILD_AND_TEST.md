# Build, test, flash, monitor and log

Each folder contains **both** firmware projects: receiver at the root and matching controller under `controller/`. It does not depend on the sibling variant or the original working folder. Keep `LICENSE` with source redistributions.

## Environment and fresh builds

The split builds were checked with **ESP-IDF 5.5.6**, ESP32 target, and its Python 3.11 environment. Install that IDF release and its toolchain using Espressif's setup procedure, then activate its `export.sh`. Do not run the project with an arbitrary system Python and assume the IDF dependencies match. See [Espressif's v5.5 build-system guide](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32/api-guides/build-system.html) for `idf.py` and project configuration.

Run from the extracted variant root:

```bash
source /path/to/esp-idf/export.sh
export IDF_TARGET=esp32
idf.py build
(cd controller && idf.py build)
python3 tools/check_variant.py
python3 tools/run_host_tests.py
python3 tools/airtime.py
```

The receiver automatically loads `sdkconfig.defaults` plus `boards/receiver-current.defaults`; controller loads its defaults plus `boards/controller-current.defaults`. `PROJECT_GPS_SUPPORT` differs by variant. The receiver's GPS flags additionally control the UART task and compiled reply path. No generated local `sdkconfig` is in the repository: clean builds apply these defaults.

Existing configurations are authoritative. To apply edited defaults, back up/move the corresponding generated `sdkconfig` first and run `idf.py reconfigure`, or change the existing configuration with `menuconfig`. Avoid reusing one variant's `sdkconfig` in the other. Pin checks:

```bash
python3 tools/check_pins.py receiver sdkconfig
python3 tools/check_pins.py controller controller/sdkconfig
```

Receiver and controller share their own copy of `components/lora_protocol/include/lora_pair.h`. Change pair/frequency/sync together and rebuild/reflash both. The control-only branch uses pair **0xff11**; the GPS branch uses **0xff10**. This protects normal cross-pair acceptance, not RF coexistence. Flash both boards from the same branch to use its matching pair ID.

ESP-IDF downloads managed components on a clean build. Root `dependencies.lock` records the tested ESP-NOW 2.5.3/CMake-utilities 1.4.0 dependencies inherited from upstream, even though the selected runtime transport is LoRa. IDF/toolchain and downloaded components are not bundled in the repository. Only ESP32 on IDF 5.5.6 is checked here; other upstream targets/releases need separate validation.


## Flash and inspect serial

Select actual ports on the receiving laptop; `/dev/ttyUSB0` and `/dev/ttyACM0` are examples, not fixed identities. Close any process owning a port before flashing or opening another reader. Opening/flashing serial can reset the ESP32 and clear arming.

```bash
# Receiver, run in variant root:
idf.py -p /dev/ttyUSB0 -b 460800 flash
idf.py -p /dev/ttyUSB0 monitor
# Controller, in a second terminal:
cd controller
idf.py -p /dev/ttyACM0 -b 460800 flash
idf.py -p /dev/ttyACM0 monitor
```

Use Ctrl+] to leave an IDF monitor. Root receiver flash configuration matches the earlier tested 2 MiB image layout with DIO mode; controller is configured for 4 MiB. Verify physical chip/module flash capacity before changing partition/flash settings. Both receiver variants retain motor-output inhibit and disabled motor startup tests.

Optional dashboard (from variant root, with the selected port free):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r tools/requirements.txt
python3 tools/serial_watch.py receiver --port /dev/ttyUSB0
# Or, in a separate terminal for the other port:
python3 tools/serial_watch.py controller --port /dev/ttyACM0
```

The dashboard requires a terminal with Python curses support; on Windows use an IDF serial monitor instead unless curses support is installed. Q closes the dashboard. It appends raw serial lines to `logs/live-receiver.log` / `logs/live-controller.log`. The log directory is local to this extracted variant.

### View the control packet and GPS timing

Open both dashboards with each board's serial port. On the controller, `TX8` shows the actual 8-byte control payload. On the receiver, `CRTP data` shows the 14 data bytes reconstructed from its logged roll, pitch, yaw and thrust values. The CRTP header adds one byte (`0x30` for port 3, channel 0). The receiver also displays `CRTP transport: paired-lora` when its boot line is captured.

On the receiver, **GPS TX gap** is the device time between consecutive successful GPS transmissions. On the controller, **GPS RTT** runs from the start of the requesting control transmission to receipt of its matched 18-byte GPS reply. **RX gap** is the time between GPS replies received by the controller and can grow when a reply is missed. The two gaps use their respective board's clock. Each needs two GPS events before appearing. The displays use existing serial messages and work with the packaged firmware.

For timestamped drone logging, keep its dashboard running and use another terminal:

```bash
python3 tools/log_drone.py --output logs/drone-session.log
```

Ctrl+C stops this follower; close the dashboard too if you want to stop raw logging. Output includes laptop local timestamps with UTC offset, device uptime where present, raw lines and observed link/sensor/arming changes. GPS-enabled logs may contain location data. Event resolution follows the approximately 1 Hz receiver status; timeout-counter increments can reveal losses between snapshots but cannot reconstruct their exact times. USB/monitor silence is logged as serial-data loss and remote state becomes unknown, rather than claiming LoRa failed. Nothing is written to onboard flash/SD. The dashboard does not automatically reconnect after USB loss.

## Verification and timing rerun

`tools/run_host_tests.py` builds four portable C suites with AddressSanitizer/UBSan: compact codec/guard, GPS codec, NMEA parser and dedicated LED invariants/deadlines/timer wrap. GPS tests are retained as common-code regression tests even in the no-GPS variant; they do not enable GPS in firmware.

After both boards are flashed with one matching variant, close dashboards and run the portable integration exercise:

```bash
python3 tools/verify_hardware.py --controller-port /dev/ttyACM0 --receiver-port /dev/ttyUSB0
```

It requires the paired receiver to advertise physical motor outputs inhibited. It exercises nonzero diagnostic controls, arming, kill, wrong pairing, replay, silence, CRTP echo and recovery; keep the motorless profile. It writes `logs/hardware-events.jsonl` and `logs/hardware-summary.json` (overwriting those two previous exercise files). Its terminal timing is not a synchronized one-way latency measurement. The hardware script exercises the installed boards and writes a local capture.

For a new latency comparison, capture `TX8` intervals and matching `LINK_ACK.rtt_us` for at least a documented duration. Compare identical PHY settings and test conditions, with GPS returns absent/present. The GPS version can toggle `GPS_RETURN 0/1` on receiver USB for a return-only A/B test; the no-GPS build keeps radio return off even if `GPS_RETURN 1` is requested. For end-to-end latency use a shared hardware timing reference as described in REPORT.md. Record firmware hashes, sample counts, losses, min/median/p95/max and the distinction between control/status/GPS packets.
