#!/usr/bin/env python3
"""Exercise the real paired LoRa link. Requires advertised no-motor bench firmware."""
import argparse
import json
import re
import threading
import time
from pathlib import Path
import serial

ROOT=Path(__file__).resolve().parent.parent/'logs'
ROOT.mkdir(exist_ok=True)
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--controller-port',required=True)
parser.add_argument('--receiver-port',required=True)
ARGS=parser.parse_args()


class Bench:
    def __init__(self):
        self.phase='boot';self.rows=[];self.stop=False;self.ports={};self.threads=[]
        self.start=time.monotonic();self.log=(ROOT/'hardware-events.jsonl').open('w')
        for name,port in [('controller',ARGS.controller_port),('receiver',ARGS.receiver_port)]:
            s=serial.Serial(port=None,baudrate=115200,timeout=.1)
            s.dtr=False;s.rts=False;s.port=port;s.open();self.ports[name]=s
            t=threading.Thread(target=self.reader,args=(name,s),daemon=True);self.threads.append(t);t.start()

    def reader(self,name,s):
        while not self.stop:
            try: line=s.readline().decode(errors='replace').strip()
            except (serial.SerialException,OSError,TypeError):return
            if not line:continue
            row={'t':time.monotonic()-self.start,'phase':self.phase,'board':name,'line':line}
            if line.startswith(('LORA_TX {','LORA_RX {')):
                try:row['data']=json.loads(line.split(' ',1)[1])
                except json.JSONDecodeError:pass
            self.rows.append(row);self.log.write(json.dumps(row)+'\n');self.log.flush()

    def cmd(self,line):
        print('CMD',line,flush=True)
        self.ports['controller'].write((line+'\n').encode())
        self.ports['controller'].flush()

    def phase_wait(self,name,duration):
        self.phase=name;print('PHASE',name,flush=True);time.sleep(duration)

    def samples(self,name=None,board='receiver'):
        return [r['data'] for r in self.rows if r['board']==board and 'data' in r and (name is None or r['phase']==name)]

    def latest(self):
        values=self.samples();assert values,'No receiver diagnostics';return values[-1]

    def wait_for(self,predicate,seconds=3):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            if predicate():return
            time.sleep(.05)
        raise AssertionError('Timed out waiting for hardware evidence')

    def exchange(self,raw,expected):
        for attempt in range(4):
            before=len(self.rows);self.cmd('CRTP '+raw)
            try:
                self.wait_for(lambda:any(r['line']=='CRTP_RX '+expected for r in self.rows[before:]),2)
                return
            except AssertionError:
                if attempt==3:raise

    def run(self):
        self.wait_for(lambda:any(x.get('bench') and x.get('linked') for x in self.samples(board='controller')),20)
        self.cmd('LIVE');self.phase_wait('baseline',5)
        assert self.latest()['armed']==0
        self.phase='crtp_echo';self.exchange('fc4c6f5261','fc4c6f5261')
        # Platform protocol query: reply is 0 + little-endian protocol version.
        self.phase='crtp_platform';before=len(self.rows);self.cmd('CRTP dd00')
        self.wait_for(lambda:any(r['line'].startswith('CRTP_RX dd00') for r in self.rows[before:]),4)
        self.phase='zero_arm';self.cmd('SET 0 0 0 0 1 2500');time.sleep(1.3)
        assert self.latest()['armed']==1
        self.phase='roll';self.cmd('SET 32 0 0 51 1 2500');time.sleep(1.4)
        r=self.latest();assert r['armed']==1 and r['roll']==8 and r['thrust']==12000,r
        self.phase='pitch_yaw';self.cmd('SET 0 -24 15 51 1 2500');time.sleep(1.4)
        r=self.latest();assert r['pitch']==-6 and r['yaw']==30 and r['thrust']==12000,r
        self.phase='kill';self.cmd('SET 0 0 0 255 3 1500');time.sleep(1.2)
        r=self.latest();assert not r['armed'] and r['thrust']==0,r
        self.cmd('LIVE');self.phase_wait('recovered',1.5)
        base=self.latest();self.phase='wrong_pair';self.cmd('PAIR 0001');time.sleep(.7);self.cmd('LIVE');time.sleep(1.5)
        r=self.latest();assert r['wrong_pair']>base['wrong_pair'] and not r['armed'],r
        base=self.latest();self.phase='duplicates';self.cmd('SEQ 1');time.sleep(.7);self.cmd('LIVE');time.sleep(1.5)
        r=self.latest();assert r['stale']>base['stale'] and not r['armed'],r
        base=self.latest();self.phase='old_sequence';self.cmd('SEQ 2');time.sleep(.65);self.cmd('LIVE');time.sleep(1.5)
        assert self.latest()['stale']>base['stale']
        self.phase='arm_before_loss';self.cmd('SET 0 0 0 0 1 2500');time.sleep(1.2)
        self.cmd('SET 16 0 0 51 1 2500');time.sleep(1.2)
        assert self.latest()['armed']==1 and self.latest()['thrust']==12000
        before=self.latest();self.phase='loss';self.cmd('PAUSE 1200');time.sleep(1.1)
        r=self.latest();assert r['timeouts']>before['timeouts'] and not r['armed'] and r['thrust']==0,r
        self.cmd('LIVE');self.phase_wait('final_live_disarmed',3)
        r=self.latest();assert not r['armed'] and r['thrust']==0
        baseline=self.samples('baseline')
        tx=[r for r in self.rows if r['phase']=='baseline' and r['line'].startswith('TX8 ')]
        periods=[(b['t']-a['t'])*1000 for a,b in zip(tx,tx[1:])]
        import statistics
        summary={'completed':True,'final_receiver':r,'baseline_command_frames':len(tx),
            'baseline_interval_ms':{'median':statistics.median(periods),'min':min(periods),'max':max(periods)},
            'baseline_receiver_samples':baseline,'checks':['bidirectional link','CRTP echo','platform query',
            'zero-throttle arm','roll/thrust expansion','pitch/yaw expansion','kill','wrong pair',
            'duplicate rejection','old sequence rejection','loss disarm/zero','recovery stays disarmed']}
        (ROOT/'hardware-summary.json').write_text(json.dumps(summary,indent=2))
        print(json.dumps(summary,indent=2),flush=True)

    def close(self):
        self.cmd('LIVE');time.sleep(.15);self.stop=True
        for t in self.threads:t.join(.3)
        for s in self.ports.values():s.close()
        self.log.close()


def main():
    b=Bench()
    try:b.run()
    finally:b.close()

if __name__=='__main__':main()
