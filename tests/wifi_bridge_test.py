import importlib.util
from pathlib import Path
import struct
import unittest

path = Path(__file__).resolve().parents[1] / 'tools/wifi_crtp_bridge.py'
spec = importlib.util.spec_from_file_location('bridge', path)
bridge = importlib.util.module_from_spec(spec);spec.loader.exec_module(bridge)


class OriginalWifiPackets(unittest.TestCase):
    def test_original_datagram_preserves_every_crtp_byte(self):
        for header in (0x30, 0x3c):
            for thrust in (0, 1, 999, 12345, 60000, 65535):
                raw = struct.pack('<BfffH', header, 8.123456, -6.234567, 30.345678, thrust)
                datagram = raw + bytes([sum(raw) & 255])
                self.assertEqual(bridge.udp_to_serial(datagram), b'COMMAND ' + raw.hex().encode() + b'\n')

    def test_parameter_and_echo_headers_unchanged(self):
        raw = bytes.fromhex('fc4c6f5261')
        self.assertEqual(bridge.udp_to_serial(raw+bytes([sum(raw)&255])), b'CRTP fc4c6f5261\n')
        self.assertEqual(bridge.serial_to_udp(b'CRTP_RX fc4c6f5261'),raw+bytes([sum(raw)&255]))

    def test_malformed_packets(self):
        for data in (b'', b'\x30', b'\x30\x01', b'\x30'*33):
            with self.assertRaises(ValueError):bridge.udp_to_serial(data)
        raw = struct.pack('<BfffH',0x3c,float('nan'),0,0,0)
        with self.assertRaises(ValueError):bridge.udp_to_serial(raw+bytes([sum(raw)&255]))
        self.assertIsNone(bridge.serial_to_udp(b'GPS_REMOTE {}'))
        self.assertIsNone(bridge.serial_to_udp(b'CRTP_RX nothex'))


if __name__ == '__main__':
    unittest.main()
