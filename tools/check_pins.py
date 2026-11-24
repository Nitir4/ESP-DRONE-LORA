#!/usr/bin/env python3
"""Check actual generated ESP32 sdkconfig wiring before flashing a changed profile."""
import argparse
from pathlib import Path


def check(path, role):
    cfg = {}
    for line in Path(path).read_text().splitlines():
        if line.startswith('CONFIG_') and '=' in line:
            key, value = line.split('=', 1)
            cfg[key] = value
    uses = {}
    valid = set(range(20)) | {21, 22, 23, 25, 26, 27} | set(range(32, 40))
    adc = {0, 2, 4, 12, 13, 14, 15, 25, 26, 27} | set(range(32, 40))

    def add(key, label, output=True, analog=False):
        pin = int(cfg[key])
        if pin < 0:
            return
        if pin not in valid or pin in range(6, 12) or pin in (1, 3):
            raise ValueError(f'{label}: GPIO{pin} invalid or reserved for flash/USB serial')
        if output and pin >= 34:
            raise ValueError(f'{label}: GPIO{pin} cannot be an output')
        if analog and pin not in adc:
            raise ValueError(f'{label}: GPIO{pin} has no ADC channel')
        if pin in uses:
            raise ValueError(f'GPIO{pin} conflict: {uses[pin]} / {label}')
        uses[pin] = label

    if role == 'controller' or cfg.get('CONFIG_TRANSPORT_LORA') == 'y':
        for name in ('NSS', 'SCK', 'MISO', 'MOSI', 'RST', 'DIO0'):
            add('CONFIG_BOARD_LORA_' + name, 'LoRa ' + name, name not in ('MISO', 'DIO0'))
    if role == 'controller':
        for name in ('ROLL', 'PITCH', 'YAW', 'THROTTLE'):
            add('CONFIG_BOARD_CTRL_' + name, name, False, True)
        for name in ('ARM_SW', 'STOP_SW'):
            add('CONFIG_BOARD_CTRL_' + name, name, False)
        if cfg.get('CONFIG_PROJECT_STATUS_LEDS') == 'y':
            for name in ('ACTIVITY', 'STATUS'):
                add('CONFIG_BOARD_CTRL_LED_' + name, 'LED ' + name)
    else:
        add('CONFIG_I2C0_PIN_SDA', 'MPU SDA')
        add('CONFIG_I2C0_PIN_SCL', 'MPU SCL')
        add('CONFIG_MPU_PIN_INT', 'MPU INT', False)
        if cfg.get('CONFIG_GPS_ENABLED') == 'y':
            add('CONFIG_GPS_RX_GPIO', 'GPS TX -> ESP RX', False)
        for color in ('BLUE', 'GREEN', 'RED'):
            add('CONFIG_LED_PIN_' + color, color + ' LED')
        if cfg.get('CONFIG_WIFI_BENCH_NO_MOTORS') != 'y':
            for motor in range(1, 5):
                add(f'CONFIG_MOTOR0{motor}_PIN', f'Motor {motor}')
    return uses


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=('controller', 'receiver'))
    parser.add_argument('sdkconfig')
    args = parser.parse_args()
    try:
        pins = check(args.sdkconfig, args.role)
    except (ValueError, KeyError) as error:
        parser.exit(1, f'Pin check failed: {error}\n')
    print(f'PASS: {args.role}, {len(pins)} active GPIO assignments, no conflicts')
