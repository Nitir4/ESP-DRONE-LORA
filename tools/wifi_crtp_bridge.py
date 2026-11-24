#!/usr/bin/env python3
"""Accept original ESP-Drone UDP application packets and bridge their CRTP over LoRa.

Run on the laptop, not on either ESP32. Point the existing UDP client to this
endpoint. Physical arming is still required; this does not bypass the guards.
"""
import argparse
import math
import socket
import struct
import time


def udp_to_serial(datagram):
    """Remove only Wi-Fi's sum8; preserve every original CRTP byte."""
    if not 2 <= len(datagram) <= 32 or sum(datagram[:-1]) & 255 != datagram[-1]:
        raise ValueError('Invalid ESP-Drone UDP length/checksum')
    crtp = datagram[:-1]
    port = crtp[0] >> 4
    if port == 3:
        if len(crtp) != 15 or crtp[0] & 3:
            raise ValueError('Only legacy port-3/channel-0 commands supported')
        r, p, y, _ = struct.unpack('<fffH', crtp[1:])
        if not all(math.isfinite(v) for v in (r, p, y)):
            raise ValueError('Nonfinite command')
        return b'COMMAND ' + crtp.hex().encode() + b'\n'
    if port in (7, 8):
        raise ValueError('Generic/high-level control ports are outside this prototype')
    return b'CRTP ' + crtp.hex().encode() + b'\n'


def serial_to_udp(line):
    if not line.startswith(b'CRTP_RX '):
        return None
    try:
        crtp = bytes.fromhex(line.split()[1].decode('ascii'))
    except (ValueError, IndexError, UnicodeError):
        return None
    if not 1 <= len(crtp) <= 31:
        return None
    return crtp + bytes([sum(crtp) & 255])


def main():
    import serial
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='Controller USB serial port')
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--udp-port', type=int, default=2390)
    args = parser.parse_args()
    with serial.Serial(args.port, 115200, timeout=0) as board, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        udp.bind((args.bind, args.udp_port));udp.setblocking(False)
        print(f'UDP {args.bind}:{args.udp_port} -> controller {args.port}. Close other serial readers.', flush=True)
        print('Keep controller sticks centred; arm physically at zero throttle. Commands require confirmed arming.', flush=True)
        peer = None
        pending = bytearray()
        while True:
            # Drain a burst and forward only its newest control command; never backlog old setpoints.
            latest_control = None
            for _ in range(256):
                try:
                    payload, sender = udp.recvfrom(1024)
                except BlockingIOError:
                    break
                try:
                    command = udp_to_serial(payload)
                except ValueError as error:
                    print(error, flush=True);continue
                if peer is not None and sender != peer:
                    continue
                peer = sender
                if command.startswith(b'COMMAND '):
                    latest_control = command
                else:
                    board.write(command)
            if latest_control is not None:
                board.write(latest_control)
            pending.extend(board.read(board.in_waiting))
            while b'\n' in pending:
                line, _, pending = pending.partition(b'\n')
                reply = serial_to_udp(line.strip())
                if reply is not None and peer is not None:
                    udp.sendto(reply, peer)
                if line.startswith((b'ERR ', b'CONTROLLER_READY')):
                    print(line.decode(errors='replace'), flush=True)
            if len(pending) > 16384:
                pending.clear()
            time.sleep(.002)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
