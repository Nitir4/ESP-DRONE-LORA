# ESP-DRONE-LORA

ESP-Drone receiver firmware and a matching ESP32 joystick controller. A point-to-point SX127x LoRa link carries **8-byte control packets**. The receiver rebuilds the legacy **14-byte CRTP setpoint payload** and feeds it to ESP-Drone's flight code.

This branch enables GPS. GPS telemetry uses the existing reply opportunities and leaves the uplink at eight bytes.

| Branch | GPS | Default pair ID |
| --- | --- | --- |
| [`main`](https://github.com/Nitir4/ESP-DRONE-LORA/tree/main) | NEO-6M UART input and separate 18-byte LoRa replies | `0xff10` |
| [`control-only`](https://github.com/Nitir4/ESP-DRONE-LORA/tree/control-only) | GPS UART task and GPS replies disabled | `0xff11` |

Both branches contain the receiver at the repository root and the joystick transmitter in [`controller/`](controller/). The current board profiles target classic ESP32 and were built with ESP-IDF 5.5.6. The receiver disables physical motor outputs for bench testing.

The **8-byte control frame takes 9.024 ms of calculated airtime**. In a 30-second indoor bench run, the controller received **28 of 29** GPS replies with a **28.142 ms median round trip**. The GPS reported valid NMEA data without a satellite fix. [Indoor bench results](evidence/indoor-gps-bench-20261124.json)

- [Protocol, measurements and limitations](REPORT.md)
- [Build, flash, serial monitor and test commands](BUILD_AND_TEST.md)
- [Wiring, LEDs and controls](docs/HARDWARE_AND_OPERATIONS.md)

The project derives from [Espressif ESP-Drone at commit `9d5a4f8`](https://github.com/espressif/esp-drone/tree/9d5a4f85199ea056f40472b2248e10dcb6988b5c) and retains its [GPL-3.0 license](LICENSE) and source notices. The flight core draws on [Bitcraze Crazyflie](https://github.com/bitcraze/crazyflie-firmware/tree/2021.01). The DSP library derives from [esp32-lin](https://github.com/whyengineer/esp32-lin/tree/6fa39f4c).

This history was assembled on November 25, 2026. Both Git date fields use the reconstructed timeline in UTC so GitHub groups the commits under those dates.

The first commit imports an unchanged snapshot of Espressif ESP-Drone. Espressif, Bitcraze and the upstream authors retain credit for their source. The upstream link above points to their original commit history.
