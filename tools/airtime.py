#!/usr/bin/env python3
"""Calculated LoRa modem airtime; this does not measure end-to-end latency."""
import json
import math

def airtime_ms(payload_bytes, sf=7, bandwidth_hz=500000, coding_rate_index=1,
               preamble_symbols=8, crc=True, implicit_header=False, low_data_rate=False):
    symbol_ms = 1000 * 2**sf / bandwidth_hz
    numerator = 8*payload_bytes - 4*sf + 28 + 16*int(crc) - 20*int(implicit_header)
    denominator = 4*(sf-2*int(low_data_rate))
    payload_symbols = 8 + max(math.ceil(numerator/denominator)*(coding_rate_index+4), 0)
    return (preamble_symbols+4.25+payload_symbols)*symbol_ms

if __name__ == '__main__':
    values = {str(n): round(airtime_ms(n), 6) for n in (8, 18, 35)}
    assert values == {'8': 9.024, '18': 12.864, '35': 19.264}
    print(json.dumps({'definition': 'Calculated RF airtime, not one-way or round-trip latency',
                      'sf': 7, 'bandwidth_hz': 500000, 'coding': '4/5',
                      'preamble_symbols': 8, 'explicit_header': True, 'crc': True,
                      'payload_bytes_to_airtime_ms': values}, indent=2))
