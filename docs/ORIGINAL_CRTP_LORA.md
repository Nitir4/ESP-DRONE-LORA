# Original CRTP command transport over LoRa

## Intended behavior

The communication goal is to change the transport while keeping the command packet accepted by ESP-Drone's Crazyflie-derived CRTP handling. Original Wi-Fi application framing is:

```text
UDP payload: CRTP header (1) | roll float32 (4) | pitch float32 (4) | yaw float32 (4) | thrust uint16 (2) | Wi-Fi sum8 (1)
```

The CRTP packet itself is **15 bytes**, with a 14-byte little-endian `<fffH` setpoint. The additive checksum belongs to Wi-Fi transport, not CRTP. UDP/IP/Wi-Fi radio headers are also transport details. Original packet structure and yaw decoding are described by [Bitcraze's commander specification](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/crtp_commander/); this project's checked-in ESP-Drone decoder remains authoritative for its thrust rules.

The new normal control frame preserves those 15 bytes:

| LoRa offset | Bytes | Meaning |
|---|---:|---|
| 0–1 | 2 | Pair ID little-endian (GPS pair 0xff10) |
| 2 | 1 | Sequence modulo 256 |
| 3 | 1 | Type 0x50; ARM bit0, KILL bit1, REPLY bit2; bit3 reserved |
| 4 | 1 | Original CRTP header, including its reserved bits |
| 5–8 | 4 | Original roll float32 bytes |
| 9–12 | 4 | Original pitch float32 bytes |
| 13–16 | 4 | Original yaw float32 bytes |
| 17–18 | 2 | Original thrust uint16 bytes |

Total **19 application bytes / 152 bits**. LoRa's preamble, modem header/FEC and payload CRC add physical overhead. Both 0x30 (ESP-Drone) and 0x3c (typical cflib reserved bits) are accepted as port3/channel0 and retained exactly. Nonfinite floats, invalid header/channel/type/flags/length and wrong pairs are rejected. There is no conversion to 8-bit channel values. The full uint16 thrust range can cross the transport; original firmware thrust unlock/min/max handling still applies afterward.

The controller produces float32 joystick angles/rates directly and uint16 thrust, with the existing joystick ranges (about ±30 degrees, ±180 degrees/s, thrust up to 60000). Input calibration/deadband still limits physical input resolution; preserving float32 transport does not invent extra ADC precision. The inner raw bytes are copied unchanged at the receiver's CRTP queue boundary when the guard permits application. Normal downstream pitch/yaw interpretation, original thrust rules and controller calculations do not constitute transport byte changes.

## Guard behavior and compatibility boundary

Pairing, fresh sequence, explicit zero-thrust arming, KILL, sensor readiness and >300 ms control expiry are retained. Disarmed/KILL frames intentionally queue zero setpoint data; the CRTP header is retained. Invalid packets do not refresh the control timer. Byte preservation applies to accepted active commands; it does not bypass arming or prevent the existing commander from interpreting/clamping its input.

The 15-byte commander path is implemented here; generic port7/high-level port8 control formats are not implemented by this guarded adapter. The existing type0x30 sparse service tunnel retains original CRTP bytes for echo/platform/parameter/log services and returns CRTP bytes unchanged, but is still bounded and disarmed-only on the uplink. Full GUI TOC/high-rate logging equivalence is not claimed. The status and GPS downlinks remain separately typed LoRa packets. This is command-format compatibility with the existing ESP-Drone CRTP stack, not a drop-in radio replacement for stock Crazyflie hardware or an unchanged Wi-Fi network endpoint.

The receiver additionally accepts the legacy 8-byte type0x10 format for transition. The controller now sends type0x50/19-byte controls by default. Both builds must be updated for the new default. Earlier ZIPs, captures and historical reports remain descriptions of the compressed stage; no new hardware timings may be inferred from them.

## Original Wi-Fi packets from a laptop

The optional bridge accepts original ESP-Drone UDP application packets (CRTP + sum8), validates/removes sum8, then sends the identical CRTP bytes through the controller's serial input and new guarded LoRa path:

```bash
# IDF/venv Python with pyserial; controller port must not have another monitor open.
python3 tools/wifi_crtp_bridge.py --port /dev/ttyACM0 --bind 127.0.0.1 --udp-port 2390
```

Point a compatible existing UDP CRTP client to the laptop endpoint, e.g. `udp://127.0.0.1:2390`, instead of the drone's Wi-Fi address. Bind to a specific LAN interface only if intentionally using a remote client. Keep the physical controller sticks centred; physically arm at zero thrust and wait for receiver confirmation before external commands are accepted. The bridge does not remotely arm the board or enable motor outputs. Opening serial may reset the controller, requiring calibration and rearming.

The bridge routes port3/channel0 legacy commands as `COMMAND <15 CRTP bytes in hex>`. Receiver-side packet bytes are not reconstructed from decimal text. A live COMMAND must be refreshed faster than its **200 ms** local stream timeout; expiry disarms rather than reverting to an armed joystick command. STOP or lost return status cancels the command. Refreshing the same raw command still generates new outer sequence numbers; the command bytes remain the same.

Other supported sparse CRTP services use `CRTP <raw header/data hex>` and responses return via UDP with sum8 appended. Only one peer is selected per bridge run; restart it when changing clients. One outstanding service request is recommended. Its service path cannot provide Wi-Fi bandwidth. The bridge is host-tested; no unchanged third-party GUI session has been exercised with the new firmware. Use physical sticks normally when no bridge is running.

## Bench diagnostics and exact-byte inspection

Controller serial now prints `TX19 <device_ms> <19-byte hex>`. Receiver serial prints `CRTP_CONTROL <device_ms> <seq> <incoming_control_bytes> <actual 15-byte queued CRTP hex>`. Receiver float diagnostics use up to nine significant digits; the hex line is the authority for byte comparisons. `input_throttle` now reports received **uint16 thrust units** on the current firmware; the controller's physical input uses 0–60000. It is no longer a 0–255 compressed throttle value. `control_bytes` identifies the wire size. After safety substitution, queued CRTP data are zeros by design.

With a paired no-motor receiver, existing diagnostic `SET` retains its historical integer units for old bench exercises but converts directly into a full CRTP command before transmission. New `SETPOINT r p y thrust flags ms` accepts original float values and uint16 thrust for precision checks. `SETCRTP <15-byte raw CRTP hex> flags ms` replays exact original packet bytes for a motor-inhibited bench check, including reserved header bits. Both diagnostic commands require a fresh paired motor-inhibited receiver. Example:

```text
SETPOINT 0 0 0 0 1 4000
SETPOINT 8.123456 -6.234567 30.345678 12345 1 2500
LIVE
```

These are USB bench diagnostics, permitted only with a fresh paired receiver advertising motor-output inhibit. They are not commands to run against an armed real vehicle. Full CRTP commands still follow the original decoder's yaw inversion, so the latter's applied yaw should be about -30.345678 degrees/s. Actual MPU angles remain measurements of physical board tilt.

## Tests, timing and next hardware step

```bash
python3 tools/run_host_tests.py
python3 tests/wifi_bridge_test.py
python3 tests/original_wifi_pipeline_test.py
python3 tools/airtime.py
# ESP-IDF 5.5.6 active:
idf.py build
(cd controller && idf.py build)
python3 tools/check_variant.py
```

The C byte-preservation test checks a golden packet, header bits, all 65536 thrust values, many finite float bit patterns, malformed headers/lengths, NaN/infinity, zero-thrust arming, replay, KILL and expiry. The independent pipeline test creates the original Wi-Fi datagram using Python `struct.pack`, passes it through the real bridge plus C encoder/decoder/guard/forwarder, and checks the queued CRTP input is byte-identical. Existing compact/GPS/parser/LED tests remain. Under ptrace-based sandboxes LeakSanitizer may be unavailable; `ASAN_OPTIONS=detect_leaks=0` retains address/undefined-behavior checks while disabling unsupported leak inspection.

Calculated RF airtimes at SF7/BW500/CR4/5, explicit header, CRC, 8-symbol preamble:

| Frame | Bytes | Airtime |
|---|---:|---:|
| Earlier compressed control | 8 | 9.024 ms |
| Original CRTP alone, without pairing wrapper (not the implemented frame) | 15 | 11.584 ms |
| Current original-CRTP control, including wrapper | 19 | 12.864 ms |
| GPS return, unchanged | 18 | 12.864 ms |
| Ordinary status, unchanged | 8 | 9.024 ms |

The control change adds **3.840 ms calculated RF occupancy per uplink** compared with the earlier compact packet. A 19-byte control + 18-byte GPS return occupies 25.728 ms of calculated airtime before turnaround, CAD and software overhead. The 50–53 ms nominal control schedule is retained; this arithmetic is not a worst-case latency guarantee. Historical ~24.121 ms status RTT and ~28.142 ms GPS RTT came from 8-byte controls and do not measure the new path. Increasing both uplinks and longer replies requires measuring full exchanges, not merely adding airtime to a claimed latency.

Both boards were flashed and checked on 25 November using `tools/validate_full_crtp_hardware.py --flash --seconds 20`. The test first disabled GPS return, then replayed an original checksum-validated Wi-Fi CRTP command over LoRa using bench-only SETCRTP, verified identical 15-byte queued CRTP and actual fractional setpoints, induced link silence, verified disarm/zero/link-LED loss and recovery, then enabled GPS and repeated measurements. This raw diagnostic replay does not automate physical joystick buttons or claim an unchanged third-party GUI session has been tested.

| Current hardware result | GPS return off | GPS return on |
|---|---:|---:|
| Measured control rate | 19.410 Hz | 19.398 Hz |
| Control interval median / p95 / maximum | 51 / 53 / 53 ms | 52 / 53 / 57 ms |
| Status RTT median / p95 | 28.078 / 28.117 ms | 28.117 / 28.117 ms |
| GPS RTT median / p95 | None | 31.638 / 32.138 ms |
| GPS returns sent / received | 0 / 0 | 20 / 20 |
| Recorded CRC errors / control timeouts in normal window | 0 / 0 | 0 / 0 |

Each measurement window lasted 20 seconds. GPS NMEA was valid in all 20 reported samples of each window, with no satellite fix. These short indoor tests do not establish outdoor range, lossless operation or synchronized one-way latency. The intentionally induced timeout is outside the normal timing windows. Exact current firmware hashes and captures are recorded in [the hardware evidence](../evidence/original-crtp-hardware-20261125.json).

Original packet replay applied roll 8.123456, pitch -6.23456717, yaw -30.3456783 (the normal decoder inversion) and thrust 12345. The original and forwarded CRTP hex are both `3cadf901419381c7c0f3c3f2413930`. Full GUI/service bandwidth and physical stick/button behavior are separate checks; do not infer those from diagnostic injection. The boards were left in live joystick mode, disarmed, zero applied thrust, with GPS return enabled. Motor-output inhibit remains enabled.

For future reruns, close serial readers, use `tools/validate_full_crtp_hardware.py` (add --flash only when needed), and save fresh results rather than replacing the historical outdoor capture. Do not describe the prior 8-byte outdoor timings as measurements of the current 19-byte transport.
