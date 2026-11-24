# Wiring, LEDs and everyday operation

The settings are modular. Edit `boards/receiver-current.defaults` and `boards/controller-current.defaults` for fresh configurations, or `idf.py menuconfig` for an existing generated `sdkconfig`. Defaults do not override values already saved in `sdkconfig`. Run `tools/check_pins.py` against each actual generated config after changes.

## Current GPIO mappings

| Device/function | Controller GPIO | Receiver GPIO |
|---|---:|---:|
| LoRa NSS | 5 | 5 |
| LoRa SCK | 18 | 18 |
| LoRa MISO | 19 | 19 |
| LoRa MOSI | 23 | 23 |
| LoRa RST | 14 | 14 |
| LoRa DIO0 | 2 | 2 |
| U2/top VRX: roll | 27 | — |
| U2/top VRY: pitch | 26 | — |
| U2/top SW: stop | 25 | — |
| U1/bottom VRX: yaw | 33 | — |
| U1/bottom VRY: throttle | 32 | — |
| U1/bottom SW: arm | 13 | — |
| MPU SDA / SCL / INT | — | 22 / 21 / 27 |
| Working controller LEDs | 4 / 21 | — |
| Receiver blue / green / red LEDs | — | 26 / 25 / 33 |
| Motor definitions, outputs inhibited | — | M1=4, M2=32, M3=12, M4=13 |

GPS-enabled version only: NEO-6M **TX → ESP GPIO16**, UART2 RX at 9600 baud. GPIO17 is reserved for ESP TX/GPS RX but is not driven. The no-GPS version does not install a GPS UART task or reserve a live GPS input. A common disabled snapshot interface remains in its source.

GPIO numbering is ESP32 GPIO numbering. Joystick axes use both ADC units; these LoRa profiles do not start Wi-Fi. Review GPIO boot-strapping functions, external loads and original ESP-Drone pin reuse before rewiring. `boards/stock-esplane-reference.json` is a reference, not an automatically compatible LoRa profile. The pin checker skips inhibited motor outputs, so passing the current bench check does not validate a future motor-enabled configuration. It also does not establish electrical pull-ups, correct voltage, antenna suitability or GPS-reserved-TX conflicts.

## Dedicated link LEDs

| Board | LED | Meaning |
|---|---|---|
| Controller | GPIO21 | Steady ON for a fresh valid paired reply; OFF when radio fails, at startup, or at 500 ms without one |
| Receiver | Green GPIO25 | Steady ON for fresh valid paired controls; OFF when radio fails, at startup, or at 300 ms without one |

The controller's USB `linked` field uses its 1-second arm-watchdog window, while its dedicated LED uses the stricter 500 ms reply deadline. Brief discrepancies between those indicators are possible.

A 25 ms renderer period adds up to about 25 ms to those model deadlines, subject to task scheduling. The LEDs never pulse for axes/GPS/arming/faults and work while disarmed. Neither proves every packet arrived or that the receiver is armed. The receiver's control guard expires when age is strictly greater than 300 ms; its LED goes off at 300 ms.

Controller GPIO4 shows a repeating fault code if present; otherwise steady neutral TX activity or 1/2/3/4 short pulses for non-neutral roll/pitch/yaw/throttle. Multiple active axes cycle in 800 ms slots. That is human-speed activity, not one flash for each packet. Transmitted throttle is zero while disarmed or awaiting receiver arm confirmation. Receiver green no longer displays axis pulses; its received `axis_code` remains available in serial diagnostics.

Receiver red GPIO33 shows fault codes, or steady armed/off disarmed when there are no faults. Fault codes rotate every 3 seconds: 1 = missing/stale MPU; 2 = local radio initialization failure; 3 = stale paired link. In the GPS version, code 4 additionally means missing fresh valid GPS sentences; controller code 5 means missing fresh GPS return, so remote sensor state is unknown. In the no-GPS version these GPS faults are intentionally suppressed, and the controller cannot infer detailed MPU health from its compact status alone.

GPS version: receiver blue GPIO26 is steady with a fresh fix, slow-blinking with valid data but no fix, off without fresh valid data. Actual GPS transmission overlays 3 short pulses with fix, 2 without fix, or 1 without current valid data. No-GPS version: blue remains off. Controller GPIO21 is always link-only; it no longer reports arming/GPS bursts.

## Before testing sticks

1. Leave both sticks centred during controller boot calibration (about 1 second).
2. Wait for link LEDs and receiver MPU calibration/readiness.
3. With centred/zero throttle, hold U1/bottom switch for about 1 second, release, and confirm the receiver reports armed.
4. Move U2 for roll/pitch and U1 for yaw/throttle. Only the increasing-ADC half of the throttle axis raises thrust; orientation depends on joystick installation.
5. After a timeout, stop or reboot, deliberately arm again at zero throttle. A recovered green link does not rearm.

If values appear stuck, compare decoded `input_*`, applied `set_*`, `armed`, `age`, `timeouts`, CRC errors and `mpu_*`. Applied zero while disarmed is expected. MPU angles respond to physical tilt, not joystick input without a moving airframe.
