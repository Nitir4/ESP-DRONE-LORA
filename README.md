# ESP-DRONE-LORA: control-only

ESP-Drone receiver and matching ESP32 joystick controller, with **original CRTP command bytes carried over raw LoRa**. The current default control frame is **19 application bytes: a 4-byte LoRa wrapper plus the original 15-byte CRTP header/setpoint**. Float32 roll, pitch and yaw and uint16 thrust are preserved. The receiver copies the inner packet into ESP-Drone's existing commander queue; no int8/uint8 compression or expansion is used on this path.

This branch disables the GPS UART task and GPS return telemetry. It uses the same original-CRTP LoRa transport as main, with pair ID **0xff11**. The main branch keeps GPS independent and returns separate 18-byte telemetry. Current profiles target standard 30-pin classic ESP32-WROOM development boards and retain motor-output inhibit for bench work.

The original command packet can also come from a Wi-Fi-format UDP client via the laptop's [UDP-to-LoRa bridge](tools/wifi_crtp_bridge.py). It removes only the Wi-Fi transport checksum, preserving all CRTP bytes. It does not make unmodified Crazyflie radio hardware receive LoRa, or give LoRa Wi-Fi's throughput.

- [Current protocol, commands, tests and limits](docs/ORIGINAL_CRTP_LORA.md)
- [Build, flash and serial tools](BUILD_AND_TEST.md)
- [Wiring, LEDs and controls](docs/HARDWARE_AND_OPERATIONS.md)
- [Historical prototype report](REPORT.md): earlier 8-byte compressed stage and its measured timing; not current 19-byte timing.

**Validation of the shared transport on main/GPS firmware:** both boards were flashed and tested on 25 November. Original CRTP bytes matched across the real LoRa link; fractional float32 setpoints and uint16 thrust reached the existing commander. A 20-second GPS-off window measured 28.078 ms median status RTT. A separate 20-second GPS-on window received 20/20 GPS returns at 31.638 ms median GPS RTT. Both normal windows had zero recorded CRC errors/control timeouts; deliberately induced loss disarmed and zeroed thrust, both link LEDs went off, and recovery remained disarmed. GPS supplied valid NMEA without a satellite fix. See [hardware evidence](evidence/original-crtp-hardware-20261125.json). The unified report remains paused at the user’s request.

Calculated airtime at SF7/BW500/CR4/5 is **12.864 ms for the 19-byte control** versus 9.024 ms for the earlier 8-byte control. This is RF airtime, not measured joystick latency or a new round-trip result.

The main/GPS pair remains `0xff10`; this control-only pair uses `0xff11`. Both branches now send original 19-byte framed CRTP commands. Previously shared ZIPs retain their earlier compressed firmware. This branch has build/host checks; the physical boards currently run main/GPS firmware. This receiver accepts legacy 8-byte commands for migration, but the old receiver cannot decode new 19-byte controls. Keep GPLv3 LICENSE and upstream attribution with redistributions.

The project derives from [Espressif ESP-Drone at commit `9d5a4f8`](https://github.com/espressif/esp-drone/tree/9d5a4f85199ea056f40472b2248e10dcb6988b5c) and retains its [GPL-3.0 license](LICENSE) and source notices. The flight core draws on [Bitcraze Crazyflie](https://github.com/bitcraze/crazyflie-firmware/tree/2021.01). The DSP library derives from [esp32-lin](https://github.com/whyengineer/esp32-lin/tree/6fa39f4c).
