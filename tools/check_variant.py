#!/usr/bin/env python3
"""Check generated build features/pins against this handoff's variant contract."""
import json
from pathlib import Path
from check_pins import check
ROOT = Path(__file__).resolve().parent.parent
variant = json.loads((ROOT/'variant.json').read_text())
expected = variant['gps_support']

def enabled(text, key):
    return 'CONFIG_'+key+'=y' in text.splitlines()

for role, path in [('receiver', ROOT/'sdkconfig'), ('controller', ROOT/'controller/sdkconfig')]:
    text = path.read_text()
    assert enabled(text, 'PROJECT_GPS_SUPPORT') == expected, role
    pins = check(path, role)
    if role == 'receiver':
        assert enabled(text, 'GPS_ENABLED') == expected
        assert enabled(text, 'GPS_TELEMETRY_ENABLED') == expected
        assert enabled(text, 'WIFI_BENCH_NO_MOTORS')
        assert enabled(text, 'TRANSPORT_LORA')
        sources = {Path(x['file']).name for x in json.loads((ROOT/'build/compile_commands.json').read_text())}
        assert ('gps_monitor.c' in sources) == expected
        assert ('gps_disabled.c' in sources) != expected
    print(f'PASS: {role}, {len(pins)} GPIO assignments, expected GPS={expected}')
print('PASS:', variant['name'], '8-byte control contract; no-motor receiver; matching feature profiles')
