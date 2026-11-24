#!/usr/bin/env python3
"""Read-only, live serial monitor for either LoRa board. Press Q to close."""
import argparse
import collections
import curses
import json
import re
import struct
import time
from pathlib import Path

import serial

PORTS = {
    'controller': '/dev/ttyACM0',
    'receiver': '/dev/ttyUSB0',
}


def monitor(screen, role):
    curses.curs_set(0)
    screen.nodelay(True)
    s = serial.Serial(port=None, baudrate=115200, timeout=0)
    s.dtr = False
    s.rts = False
    s.port = PORTS[role]
    s.open()
    recent = collections.deque(maxlen=12)
    partial = bytearray()
    status = {}
    gps = {}
    board = {}
    ack = {}
    gps_ack = {}
    gps_rx_interval = None
    gps_tx = None
    gps_tx_interval = None
    gps_updated = 0
    transport = None
    logdir = Path(__file__).resolve().parent.parent / 'logs'
    logdir.mkdir(exist_ok=True)
    snapshot = logdir / f'live-{role}.json'
    logfile = (logdir / f'live-{role}.log').open('a', buffering=1)
    control = None
    crtp_control = None
    updated = 0
    last_draw = 0
    try:
        while True:
            if screen.getch() in (ord('q'), ord('Q')):
                return
            data = s.read(min(16384, s.in_waiting))
            if data:
                partial.extend(data)
                while b'\n' in partial:
                    raw, _, partial = partial.partition(b'\n')
                    line = raw.decode(errors='replace').strip()
                    line = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', line)
                    line = ''.join(c if c.isprintable() else ' ' for c in line)
                    if not line:
                        continue
                    recent.append(line[-300:])
                    logfile.write(line + '\n')
                    updated = time.monotonic()
                    if 'CRTP transport: ' in line:
                        transport = line.split('CRTP transport: ', 1)[1]
                        gps_tx = gps_tx_interval = gps_rx_interval = None
                        gps_ack = {}
                    if line.startswith(('LORA_TX {', 'LORA_RX {')):
                        try:
                            status = json.loads(line.split(' ', 1)[1])
                            snapshot.write_text(json.dumps(status, indent=2) + '\n')
                        except json.JSONDecodeError:
                            pass
                    if line.startswith('GPS_STATUS {'):
                        try:
                            candidate = json.loads(line.split(' ', 1)[1])
                            if not all(key in candidate for key in ('data', 'fix', 'latitude', 'longitude')):
                                continue
                            gps = candidate
                            gps_updated = time.monotonic()
                            snapshot.with_name('live-gps.json').write_text(json.dumps(gps, indent=2) + '\n')
                        except json.JSONDecodeError:
                            pass
                    if line.startswith(('GPS_REMOTE {', 'BOARD_STATUS {', 'LINK_ACK {')):
                        try:
                            candidate = json.loads(line.split(' ', 1)[1])
                            if line.startswith('GPS_REMOTE {'):
                                if not all(k in candidate for k in ('data', 'fix', 'latitude', 'longitude')):
                                    continue
                                gps=candidate;gps_updated=time.monotonic()
                                snapshot.with_name('live-remote-gps.json').write_text(json.dumps(gps, indent=2)+'\n')
                            elif line.startswith('BOARD_STATUS {'):
                                board=candidate
                                snapshot.with_name(f'live-{role}-board.json').write_text(json.dumps(board, indent=2)+'\n')
                            else:
                                if candidate.get('kind') == 0x40:
                                    gps_rx_interval = (
                                        candidate['ms'] - gps_ack['ms']
                                        if gps_ack and candidate['ms'] >= gps_ack['ms'] else None
                                    )
                                    gps_ack = candidate
                                ack = candidate
                        except json.JSONDecodeError:
                            pass
                    if line.startswith('GPS_TX '):
                        try:
                            _, stamp, sequence, frame = line.split()
                            stamp = int(stamp)
                            payload = bytes.fromhex(frame)
                            if len(payload) == 18 and payload[3] & 0xf0 == 0x40:
                                gps_tx_interval = stamp - gps_tx['ms'] if gps_tx and stamp >= gps_tx['ms'] else None
                                gps_tx = {'ms': stamp, 'seq': int(sequence), 'bytes': len(payload)}
                        except ValueError:
                            pass
                    if line.startswith('CRTP_CONTROL '):
                        try:
                            _, stamp, seq, size, hexdata = line.split()
                            packet = bytes.fromhex(hexdata)
                            if len(packet) == 15:
                                crtp_control = packet
                        except ValueError:
                            pass
                    if line.startswith(('TX8 ', 'TX19 ')):
                        try:
                            payload = bytes.fromhex(line.split()[2])
                            if len(payload) == 19 and payload[3] & 0xf0 == 0x50:
                                r, p, y, thrust = struct.unpack('<fffH', payload[5:])
                                control = dict(seq=payload[2], flags=payload[3] & 7, roll=r, pitch=p,
                                               yaw=y, throttle=thrust, hex=payload.hex(' '), bytes=19)
                            elif len(payload) == 8:
                                signed = lambda x: x-256 if x>127 else x
                                control = dict(seq=payload[2], flags=payload[3] & 7,
                                               roll=signed(payload[4])*.25, pitch=signed(payload[5])*.25,
                                               yaw=signed(payload[6])*2, throttle=payload[7], hex=payload.hex(' '), bytes=8)
                        except (ValueError, IndexError, struct.error):
                            pass
            if len(partial) > 16384:
                partial.clear()
            now = time.monotonic()
            if now - last_draw < .1:
                time.sleep(.01)
                continue
            last_draw = now
            screen.erase()
            height, width = screen.getmaxyx()

            def put(y, text, bold=False):
                if 0 <= y < height - 1:
                    try:
                        screen.addnstr(y, 1, str(text), max(1, width - 3),
                                       curses.A_BOLD if bold else curses.A_NORMAL)
                    except curses.error:
                        pass

            put(0, f'LoRa {role.upper()} | 115200 baud | Q closes this monitor', True)
            put(1, 'LIVE SERIAL DATA - physical motor outputs are DISABLED')
            put(2, f'Last serial data: {now - updated:.1f}s ago' if updated else 'Waiting for board startup and calibration...')
            if role == 'controller':
                put(4, 'LoRa control: 19 B = 4 B wrapper + original 15 B CRTP command', True)
                put(5, 'Centre throttle, hold U1 button 1s to ARM. U2 button STOPS.')
                put(6, 'U1 / bottom: yaw + throttle     U2 / top: roll + pitch')
                put(7, f"Link: {'CONNECTED' if status.get('linked') else 'WAITING'}   "
                       f"Arm request: {'ON' if status.get('armed') else 'OFF'}   "
                       f"Sent: {status.get('sent', 0)}   Replies: {status.get('replies', 0)}", True)
                if control:
                    put(9, f"Transmitted: roll {control['roll']:+.6g} deg   pitch {control['pitch']:+.6g} deg")
                    put(10, f"Yaw {control['yaw']:+.6g} deg/s   thrust {control['throttle']}   "
                            f"sequence {control['seq']}")
                    put(11, f"Frame arm={int(bool(control['flags'] & 1))}  kill={int(bool(control['flags'] & 2))}  "
                            f"reply requested={int(bool(control['flags'] & 4))}")
                    put(13, f"Actual {control['bytes']}-byte packet: " + control['hex'])
                peer_state = ('YES' if status.get('receiver_armed') else 'NO') if status.get('receiver_status_fresh') else 'WAITING'
                put(12, f'Drone confirmed armed: {peer_state}', True)
                put(14, f"U1 SW: {'PRESSED' if status.get('arm_button') else 'released'}  "
                        f"U2 STOP: {'PRESSED' if status.get('stop_button') else 'released'}  "
                        f"hold {status.get('hold_ms', 0)} ms")
                put(15, f"Arming: {status.get('arm_state', 'waiting for diagnostics')}  "
                        f"stick thrust {status.get('input_throttle', '?')}/60000", True)
                put(16, f"MPU return: roll {status.get('mpu_roll', 0):+d} deg  "
                        f"pitch {status.get('mpu_pitch', 0):+d} deg  "
                        f"RSSI {status.get('rssi', 0)} dBm")
                put(17, 'Stick axes transmit while disarmed; receiver applies zero until armed.')
                if gps:
                    valid=now-gps_updated<3 and gps.get('fix')
                    put(19, f"GPS over LoRa: {'FIX' if valid else 'NO FRESH FIX'}  "
                            f"received {gps.get('received', 0)}  sats {gps.get('satellites', 0)} / "
                            f"view {gps.get('in_view', 0)}", True)
                    put(20, f"Lat {gps['latitude']:.7f}  Lon {gps['longitude']:.7f}  "
                            f"Alt {gps.get('altitude', 0)} m" if valid else 'Waiting for valid GPS telemetry...')
                else:
                    put(19, 'GPS over LoRa: waiting for separate 18-byte telemetry', True)
                if ack:
                    put(21, f"Last reply RTT: {ack['rtt_us']/1000:.2f} ms  "
                            f"reply payload {ack['bytes']} bytes")
                if gps_ack:
                    interval = f'{gps_rx_interval} ms' if gps_rx_interval is not None else 'waiting'
                    put(22, f"Last GPS reply: {gps_ack['bytes']} B | RTT {gps_ack['rtt_us']/1000:.2f} ms | RX gap {interval}")
            else:
                if transport:
                    put(4, f'CRTP transport reported by receiver: {transport}', True)
                else:
                    put(4, 'LoRa control: 19 B = 4 B wrapper + original 15 B CRTP command', True)
                put(6, f"Armed: {'YES' if status.get('armed') else 'NO'}   "
                       f"Accepted: {status.get('ok', 0)}   Replies: {status.get('tx', 0)}", True)
                put(8, f"Received roll {status.get('input_roll', 0):+.6g} deg   "
                       f"pitch {status.get('input_pitch', 0):+.6g} deg", True)
                put(9, f"Received yaw {status.get('input_yaw', 0):+.2f} deg/s   "
                       f"wire thrust {status.get('input_throttle', 0)}")
                put(11, f"Applied setpoint: roll {status.get('set_roll', 0):+.2f} deg   "
                        f"pitch {status.get('set_pitch', 0):+.2f} deg", True)
                put(12, f"Setpoint yaw {status.get('set_yaw', 0):+.2f} deg/s   "
                        f"thrust {status.get('set_thrust', 0):.0f}/60000")
                if crtp_control is not None:
                    put(13, 'Actual queued CRTP (15 B): ' + crtp_control.hex(' '))
                elif all(field in status for field in ('roll', 'pitch', 'yaw', 'thrust')):
                    payload = struct.pack('<fffH', status['roll'], status['pitch'], status['yaw'], status['thrust'])
                    put(13, 'CRTP data (14 B, from diagnostics): ' + payload.hex(' '))
                put(14, f"Real MPU: roll {status.get('mpu_roll', 0):+.2f} deg   "
                        f"pitch {status.get('mpu_pitch', 0):+.2f} deg", True)
                put(15, f"RSSI {status.get('rssi', 0)} dBm   SNR {status.get('snr_q', 0) / 4:.2f} dB   "
                        f"control age {status.get('age', 0)} ms")
                put(16, f"Rejected: other pair {status.get('wrong_pair', 0)}   "
                        f"stale {status.get('stale', 0)}   CRC {status.get('crc', 0)}   "
                        f"timeouts {status.get('timeouts', 0)}")
                put(17, 'ARMED: commands enabled.' if status.get('armed') else
                        'DISARMED: received axes are visible; ESP-Drone commands are held at zero.')
                put(18, 'MPU measures physical tilt; it does not follow sticks without motors.')
                if gps_tx:
                    interval = f'{gps_tx_interval} ms' if gps_tx_interval is not None else 'waiting'
                    put(19, f"Last GPS TX: {gps_tx['bytes']} B | sequence {gps_tx['seq']} | TX gap {interval}")
                if not gps:
                    put(20, 'GPS: waiting for GPS diagnostics from receiver firmware', True)
                else:
                    gps_fresh = now - gps_updated < 3
                    gps_state = (('FIX' if gps.get('fix') else 'NMEA received / NO FIX') if gps.get('data') else 'NO VALID NMEA') if gps_fresh else 'STALE SERIAL DATA'
                    put(20, f"GPS: {gps_state} | satellites used {gps.get('satellites', 0)} / "
                            f"in view {gps.get('in_view', 0)}", True)
                    put(21, f"UART RX{gps.get('rx', '?')} @ {gps.get('baud', 9600)} | bytes {gps.get('bytes', 0)} | "
                            f"NMEA {gps.get('sentences', 0)} | UTC {gps.get('utc') or '--'}")
                    put(22, f"Lat {gps['latitude']:.7f}  Lon {gps['longitude']:.7f}  "
                            f"Altitude {gps.get('altitude', 0):.1f} m" if gps_fresh and gps.get('fix') else
                            'Coordinates unavailable until a fresh satellite fix.')
            put(23, f"LED pattern: axis {board.get('axis_code', 0)}  fault {board.get('fault_code', 0)}  "
                    f"mask {board.get('led_mask', 0):03b} (legend in docs/HARDWARE_AND_OPERATIONS.md)")
            raw_start = 25
            put(raw_start, 'Most recent raw serial lines:', True)
            for y, line in enumerate(list(recent)[-max(0, height - raw_start - 4):], raw_start + 2):
                put(y, line)
            screen.refresh()
    finally:
        s.close()
        logfile.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('board', choices=PORTS)
    parser.add_argument('--port', help='Serial port for this board, e.g. /dev/ttyUSB0')
    args = parser.parse_args()
    if args.port:
        PORTS[args.board] = args.port
    try:
        curses.wrapper(monitor, args.board)
    except KeyboardInterrupt:
        pass
    except serial.SerialException as error:
        print('Serial monitor:', error)
        input('Press Enter to close...')
