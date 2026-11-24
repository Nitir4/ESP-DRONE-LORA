#!/usr/bin/env python3
"""Run portable protocol, parser and LED behavior tests without ESP-IDF/hardware."""
import os
import subprocess
import tempfile
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
cases = [
    ('control', 'tests/lora_protocol_test.c', 'components/lora_protocol/lora_protocol.c', 'components/lora_protocol/include'),
    ('gps-codec', 'tests/gps_frame_test.c', 'components/lora_protocol/lora_protocol.c', 'components/lora_protocol/include'),
    ('gps-parser', 'tests/gps_nmea_test.c', 'components/gps/gps_nmea.c', 'components/gps/include'),
    ('leds', 'tests/status_pattern_test.c', 'components/status_led/status_pattern.c', 'components/status_led/include'),
]
with tempfile.TemporaryDirectory(prefix='esp-drone-tests-') as folder:
    for name, test, source, inc in cases:
        exe = str(Path(folder)/name)
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=address,undefined', '-I'+inc, test, source, '-lm', '-o', exe], cwd=ROOT, check=True)
        subprocess.run([exe], cwd=ROOT, check=True)
print('PASS: four portable suites; GPS parser/codec tests also retained in the no-GPS source for regression coverage.')
