#!/usr/bin/env python3
"""Timestamp the existing drone monitor's output without opening/resetting USB."""
import argparse
import json
import signal
import time
from datetime import datetime
from pathlib import Path


class Recorder:
    def __init__(self, output):
        self.output = output
        self.states = {}
        self.device_ms = None
        self.timeouts = None
        self.last_serial = time.monotonic()
        self.serial_lost = False

    def write(self, text):
        stamp = datetime.now().astimezone().isoformat(timespec='milliseconds')
        self.output.write(f'{stamp} {text}\n')

    def event(self, name, **fields):
        if self.device_ms is not None:
            fields['device_ms'] = self.device_ms
        self.write('EVENT ' + name + ' ' + json.dumps(fields, separators=(',', ':')))

    def state(self, key, value, on, off):
        value = bool(value)
        previous = self.states.get(key)
        if previous is None or previous != value:
            self.event(on if value else off, initial=previous is None)
        self.states[key] = value

    def line(self, line):
        self.last_serial = time.monotonic()
        if self.serial_lost:
            self.event('SERIAL_DATA_RESUMED')
            self.serial_lost = False
        self.write('SERIAL ' + line)
        try:
            name, payload = line.split(' ', 1)
            data = json.loads(payload)
        except (ValueError, json.JSONDecodeError):
            return
        if not isinstance(data, dict):
            return
        if 'ms' in data:
            now = data['ms']
            if self.device_ms is not None and now < self.device_ms and self.device_ms - now > 10000:
                self.event('DRONE_REBOOT_OR_CLOCK_WRAP', new_device_ms=now)
                self.states.clear()
                self.timeouts = None
            self.device_ms = now
        if name == 'BOARD_STATUS':
            self.state('lora', data['led_mask'] & 1, 'LORA_CONNECTED', 'LORA_DISCONNECTED')
            self.state('mpu', data['mpu_ok'], 'MPU_DATA_HEALTHY', 'MPU_DATA_MISSING_OR_STALE')
            self.state('gps', data['gps_data'], 'GPS_DATA_HEALTHY', 'GPS_DATA_MISSING_OR_STALE')
            self.state('gps_fix', data['gps_fix'], 'GPS_FIX_ACQUIRED', 'GPS_FIX_UNAVAILABLE')
            self.state('gps_return', data['gps_return'], 'GPS_RETURN_ENABLED', 'GPS_RETURN_DISABLED')
        elif name == 'LORA_RX':
            # Detect losses that recovered between the roughly 1 Hz status lines.
            count = data.get('timeouts', 0)
            if self.timeouts is not None and count > self.timeouts:
                self.event('LORA_TIMEOUTS_OBSERVED', new_timeouts=count-self.timeouts,
                           total=count, note='Occurred since previous diagnostic; exact event time unavailable')
            self.timeouts = count
            self.state('armed', data['armed'], 'DRONE_ARMED', 'DRONE_DISARMED')

    def tick(self):
        age = time.monotonic() - self.last_serial
        if age >= 3 and not self.serial_lost:
            self.event('SERIAL_DATA_LOST', silence_seconds=round(age, 3),
                       note='USB/monitor data unavailable; LoRa and sensor state now unknown')
            self.serial_lost = True
            self.states.clear()
            self.timeouts = None


def follow(source, output):
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    with output.open('a', buffering=1) as target, source.open() as incoming:
        incoming.seek(0, 2)
        recorder = Recorder(target)
        recorder.event('LOG_STARTED', source=str(source),
                       time_basis='Laptop local time with UTC offset; device_ms is drone uptime',
                       resolution='Status diagnostics approximately 1 Hz; raw lines also retained')
        partial = ''
        while running:
            chunk = incoming.read()
            if chunk:
                partial += chunk
                lines = partial.split('\n')
                partial = lines.pop()
                for line in lines:
                    if line.strip():
                        recorder.line(line.strip())
            recorder.tick()
            time.sleep(.03)
        recorder.event('LOG_STOPPED')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parent.parent / 'logs/live-receiver.log')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    follow(args.source, args.output)
