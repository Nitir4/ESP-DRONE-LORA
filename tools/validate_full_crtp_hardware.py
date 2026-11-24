#!/usr/bin/env python3
"""Flash/check the attached motor-inhibited prototype; save separate dated evidence."""
import argparse
import concurrent.futures
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import statistics
import struct
import subprocess
import sys
import threading
import time

import serial
from wifi_crtp_bridge import udp_to_serial

ROOT = Path(__file__).resolve().parents[1]
PORTS = {
    'controller': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A58019989-if00',
    'receiver': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
}


def stats(values):
    values = sorted(values)
    if not values:
        return None
    return {'count': len(values), 'min': values[0], 'median': statistics.median(values),
            'p95': values[min(len(values)-1, int(.95*len(values)))], 'max': values[-1]}


def flash(role, folder):
    build = ROOT / ('controller/build' if role == 'controller' else 'build')
    cmd = [sys.executable, '-m', 'esptool', '--chip', 'esp32', '-p', PORTS[role], '-b', '460800',
           '--before', 'default_reset', '--after', 'hard_reset', 'write_flash', '@flash_args']
    print('Flashing', role, PORTS[role], flush=True)
    with (folder/f'{role}-flash.log').open('w') as log:
        subprocess.run(cmd, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True)
    print('Flashed', role, flush=True)


class Capture:
    def __init__(self, folder):
        self.started = time.monotonic()
        self.phase = 'startup'
        self.rows = []
        self.stop = False
        self.lock = threading.Lock()
        self.ports = {}
        self.threads = []
        self.log = (folder/'events.jsonl').open('w', buffering=1)
        for role, device in PORTS.items():
            port = serial.Serial(port=None, baudrate=115200, timeout=.1)
            port.dtr = False;port.rts = False;port.port = device;port.open()
            self.ports[role] = port
            thread = threading.Thread(target=self.read, args=(role, port), daemon=True)
            self.threads.append(thread);thread.start()

    def now(self):
        return time.monotonic()-self.started

    def read(self, role, port):
        while not self.stop:
            try:
                line = port.readline().decode(errors='replace').strip()
            except (serial.SerialException, OSError, TypeError):
                return
            line = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', line)
            if not line:
                continue
            row = {'t': self.now(), 'phase': self.phase, 'board': role, 'line': line}
            if ' {' in line:
                try:
                    row['data'] = json.loads(line.split(' ', 1)[1])
                except (ValueError, json.JSONDecodeError):
                    pass
            with self.lock:
                self.rows.append(row);self.log.write(json.dumps(row)+'\n')

    def latest(self, role, tag, since=0):
        with self.lock:
            for row in reversed(self.rows):
                if row['t'] < since:
                    break
                if row['board'] == role and row['line'].startswith(tag+' {') and 'data' in row:
                    return row['data']
        return {}

    def select(self, role=None, tag=None, phase=None, since=0):
        with self.lock:
            return [r for r in self.rows if r['t'] >= since and (role is None or r['board'] == role)
                    and (tag is None or r['line'].startswith(tag+' '))
                    and (phase is None or r['phase'] == phase)]

    def wait(self, predicate, timeout=5):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if predicate():
                return
            time.sleep(.025)
        raise AssertionError({'phase': self.phase, 'controller': self.latest('controller','LORA_TX'),
                              'receiver': self.latest('receiver','LORA_RX')})

    def cmd(self, role, command):
        self.ports[role].write((command+'\n').encode());self.ports[role].flush()
        print(role, command, flush=True)

    def stage(self, name, duration):
        self.phase = name;print('PHASE',name,duration,'seconds',flush=True);time.sleep(duration)

    def metrics(self, phase):
        frames = self.select('controller','TX19',phase)
        payloads = [bytes.fromhex(r['line'].split()[2]) for r in frames]
        assert len(payloads) > 100 and all(len(p)==19 and p[3]&0xf0==0x50 for p in payloads)
        stamps = [int(r['line'].split()[1]) for r in frames]
        gaps = [b-a for a,b in zip(stamps,stamps[1:])]
        replies = [r['data'] for r in self.select('controller','LINK_ACK',phase) if 'data' in r]
        gps_sent = self.select('receiver','GPS_TX',phase)
        gps_received = self.select('controller','GPS_REMOTE',phase)
        health = [r['data'] for r in self.select('receiver','GPS_STATUS',phase) if 'data' in r]
        txstamps = [int(r['line'].split()[1]) for r in gps_sent]
        rx = [r['data'] for r in self.select('receiver','LORA_RX',phase) if 'data' in r]
        assert len(rx) > 5
        if phase == 'gps_return_off':
            assert not gps_sent and not gps_received
        else:
            assert len(gps_sent) >= 5 and len(gps_received) >= 5
        return {'control_bytes':19,'original_crtp_bytes':15,'control_hz':1000/statistics.mean(gaps),
                'control_interval_ms':stats(gaps),
                'status_rtt_ms':stats([r['rtt_us']/1000 for r in replies if r['kind']==0x20]),
                'gps_rtt_ms':stats([r['rtt_us']/1000 for r in replies if r['kind']==0x40]),
                'gps_sent':len(gps_sent),'gps_received':len(gps_received),
                'gps_interval_ms':stats([b-a for a,b in zip(txstamps,txstamps[1:])]),
                'gps_data_samples':sum(bool(r['data']) for r in health),
                'gps_fix_samples':sum(bool(r['fix']) for r in health),
                'receiver_counter_deltas':{k:rx[-1][k]-rx[0][k] for k in ['ok','crc','timeouts','stale']}}

    def check_original_packet(self):
        self.phase = 'original_wifi_packet'
        # Start a fresh zero-thrust arm edge on the motor-inhibited receiver.
        zero = struct.pack('<BfffH',0x3c,0,0,0,0)
        self.cmd('controller',f'SETCRTP {zero.hex()} 0 3000')
        since=self.now()
        self.wait(lambda: self.latest('receiver','LORA_RX',since).get('armed')==0)
        self.cmd('controller',f'SETCRTP {zero.hex()} 1 10000')
        since=self.now()
        self.wait(lambda: self.latest('receiver','LORA_RX',since).get('armed')==1)
        original = struct.pack('<BfffH',0x3c,8.123456,-6.234567,30.345678,12345)
        original_udp = original+bytes([sum(original)&255])
        # Real Wi-Fi framing goes through the same bridge parser; bench injection
        # replaces the physical button step only while motor inhibit is advertised.
        raw = bytes.fromhex(udp_to_serial(original_udp).split()[1].decode())
        assert raw==original
        since=self.now()
        self.cmd('controller',f'SETCRTP {raw.hex()} 1 10000')
        self.wait(lambda: any(bytes.fromhex(r['line'].split()[2])[4:]==original
                             for r in self.select('controller','TX19',since=since)))
        self.wait(lambda: any(r['line'].split()[-1]==original.hex()
                             for r in self.select('receiver','CRTP_CONTROL',since=since)),5)
        expected=struct.unpack('<fffH',original[1:])
        self.wait(lambda: all(abs(self.latest('receiver','LORA_RX',since).get(k,1e9)-v)<1e-5
                              for k,v in zip(['set_roll','set_pitch','set_yaw','set_thrust'],
                                             [expected[0],expected[1],-expected[2],expected[3]])))
        received=self.latest('receiver','LORA_RX',since)
        assert received['control_bytes']==19
        proof={'original_udp_hex':original_udp.hex(),'original_crtp_hex':original.hex(),
               'forwarded_crtp_hex':original.hex(),'crtp_bytes_equal':True,
               'injected_via':'SETCRTP bench-only raw replay after normal bridge UDP validation; physical joystick/button not automated',
               'actual_setpoints':{k:received[k] for k in ['set_roll','set_pitch','set_yaw','set_thrust']}}
        print('PASS real-radio byte identity and full precision',flush=True)
        self.phase='kill'
        self.cmd('controller',f'SETCRTP {original.hex()} 3 4000')
        since=self.now()
        self.wait(lambda: self.latest('receiver','LORA_RX',since).get('armed')==0 and
                  self.latest('receiver','LORA_RX',since).get('set_thrust')==0)
        self.cmd('controller','LIVE')
        self.wait(lambda: self.latest('controller','LORA_TX').get('linked'))
        return proof

    def check_loss(self):
        self.phase='arm_before_loss'
        since=self.now();self.cmd('controller','SETPOINT 0 0 0 0 1 10000')
        self.wait(lambda:self.latest('receiver','LORA_RX',since).get('armed')==1)
        since=self.now();self.cmd('controller','SETPOINT 4.125 0 0 12345 1 10000')
        self.wait(lambda:self.latest('receiver','LORA_RX',since).get('set_thrust')==12345)
        before=self.latest('receiver','LORA_RX')['timeouts']
        self.phase='link_silence';since=self.now();self.cmd('controller','PAUSE 4000')
        self.wait(lambda:self.latest('receiver','LORA_RX',since).get('timeouts',before)>before and
                  self.latest('receiver','LORA_RX',since).get('set_thrust')==0,3)
        self.wait(lambda: self.latest('receiver','BOARD_STATUS',since) and
                  not self.latest('receiver','BOARD_STATUS',since)['led_mask']&1 and
                  self.latest('controller','BOARD_STATUS',since) and
                  not self.latest('controller','BOARD_STATUS',since)['led_mask']&2,3)
        lost=self.latest('receiver','LORA_RX',since)
        self.phase='recovery';since=self.now();self.cmd('controller','LIVE')
        self.wait(lambda:self.latest('receiver','BOARD_STATUS',since).get('led_mask',0)&1 and
                  self.latest('controller','BOARD_STATUS',since).get('led_mask',0)&2,5)
        self.wait(lambda:self.latest('receiver','LORA_RX',since).get('armed')==0 and
                  self.latest('receiver','LORA_RX',since).get('set_thrust')==0)
        print('PASS link-loss zero/disarm, dedicated LEDs and disarmed recovery',flush=True)
        return {'timeouts_increased':lost['timeouts']>before,'loss_disarmed':not lost['armed'],
                'loss_thrust_zero':lost['set_thrust']==0,'both_link_leds_off':True,'recovery_disarmed':True}

    def close(self):
        try:
            self.cmd('controller','LIVE');self.cmd('receiver','GPS_RETURN 1');time.sleep(.3)
        finally:
            self.stop=True
            for thread in self.threads:thread.join(.5)
            for port in self.ports.values():port.close()
            self.log.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--flash',action='store_true')
    parser.add_argument('--seconds',type=int,default=20)
    args=parser.parse_args()
    assert 10<=args.seconds<=60
    cfg=(ROOT/'sdkconfig').read_text()
    assert 'CONFIG_GPS_ENABLED=y' in cfg, 'GPS A/B comparison requires the main/GPS profile; use verify_hardware.py for this GPS-disabled branch'
    assert 'CONFIG_WIFI_BENCH_NO_MOTORS=y' in cfg
    assert '# CONFIG_MOTORS_STARTUP_TEST is not set' in cfg
    stamp=datetime.now().astimezone().strftime('%Y%m%d-%H%M%S')
    folder=ROOT/'logs'/('full-crtp-'+stamp);folder.mkdir(parents=True)
    print('Saving capture:',folder,flush=True)
    for role,port in PORTS.items():
        assert Path(port).exists(),(role,port)
        owner=subprocess.run(['fuser',port],capture_output=True,text=True)
        assert owner.returncode==1,('Serial port already owned',role,owner.stdout,owner.stderr)
    if args.flash:
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            list(pool.map(lambda role:flash(role,folder),PORTS))
    check=Capture(folder)
    summary={'completed':False,'capture':str(folder),'flashed':args.flash,'physical_motor_outputs':False}
    try:
        check.wait(lambda:check.latest('controller','LORA_TX').get('bench') and
                   check.latest('controller','LORA_TX').get('linked') and
                   check.latest('receiver','LORA_RX').get('control_bytes')==19,35)
        check.cmd('controller','LIVE');check.cmd('receiver','GPS_RETURN 0')
        since=check.now()
        check.wait(lambda:check.latest('receiver','BOARD_STATUS',since).get('gps_return')==0,5)
        check.stage('settle_off',1.5);check.stage('gps_return_off',args.seconds)
        summary['gps_return_off']=check.metrics('gps_return_off')
        summary['byte_identity']=check.check_original_packet()
        summary['loss_checks']=check.check_loss()
        check.phase='enable_gps';since=check.now();check.cmd('receiver','GPS_RETURN 1')
        check.wait(lambda:check.latest('receiver','BOARD_STATUS',since).get('gps_return')==1)
        check.stage('settle_on',1.5);check.stage('gps_return_on',args.seconds)
        summary['gps_return_on']=check.metrics('gps_return_on')
        final=check.latest('receiver','LORA_RX');assert not final['armed'] and final['set_thrust']==0
        summary.update(completed=True,final_receiver=final,
                       timing_definition='Controller TX-start to matching reply parse; not one-way latency',
                       calculated_airtime_ms={'control_19_bytes':12.864,'status_8_bytes':9.024,'gps_18_bytes':12.864},
                       binary_sha256={str(p):hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
                                      for p in [Path('build/ESPDrone.bin'),Path('controller/build/LoRaController.bin')]})
        print('PASS:',json.dumps(summary,indent=2),flush=True)
    except Exception as error:
        summary['error']=repr(error)
        raise
    finally:
        check.close()
        (folder/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')


if __name__=='__main__':
    main()
