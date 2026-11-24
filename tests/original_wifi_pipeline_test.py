"""Independent original Wi-Fi packet -> Python bridge -> actual C codec/guard/forward."""
import ctypes as C
import importlib.util
import os
from pathlib import Path
import struct
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('bridge', ROOT/'tools/wifi_crtp_bridge.py')
bridge = importlib.util.module_from_spec(spec);spec.loader.exec_module(bridge)


class Full(C.Structure):
    _fields_ = [('raw', C.c_uint8*15), ('flags', C.c_uint8), ('seq', C.c_uint8)]


class Guard(C.Structure):
    _fields_ = [('synced', C.c_bool), ('armed', C.c_bool), ('arm_requested', C.c_bool),
                ('have_control', C.c_bool), ('last_seq', C.c_uint8), ('last_ms', C.c_uint32)]


with tempfile.TemporaryDirectory(prefix='original-wifi-crtp-') as tmp:
    so = str(Path(tmp)/'protocol.so')
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-shared', '-fPIC', '-I'+str(ROOT/'components/lora_protocol/include'),
                    str(ROOT/'components/lora_protocol/lora_protocol.c'), '-lm', '-o', so], check=True)
    lib = C.CDLL(so)
    lib.lr_encode_full_control.argtypes = [C.POINTER(C.c_uint8), C.c_uint16, C.POINTER(Full)]
    lib.lr_decode_full_control.argtypes = [C.POINTER(C.c_uint8), C.c_size_t, C.c_uint16, C.POINTER(Full)]
    lib.lr_guard_accept_full.argtypes = [C.POINTER(Guard), C.POINTER(Full), C.c_uint32]
    lib.lr_guard_expire.argtypes = [C.POINTER(Guard), C.c_uint32]
    lib.lr_full_control_forward.argtypes = [C.POINTER(C.c_uint8), C.POINTER(Full), C.c_bool]
    guard = Guard()

    def transfer(raw, flags, seq, now):
        udp = raw+bytes([sum(raw)&255])
        serial = bridge.udp_to_serial(udp)
        retained = bytes.fromhex(serial.split()[1].decode())
        control = Full((C.c_uint8*15).from_buffer_copy(retained), flags, seq)
        wire = (C.c_uint8*19)();decoded = Full();forwarded = (C.c_uint8*15)()
        lib.lr_encode_full_control(wire, 0xff10, C.byref(control))
        assert bytes(wire)[4:] == raw
        assert lib.lr_decode_full_control(wire, 19, 0xff10, C.byref(decoded)) == 0
        assert lib.lr_guard_accept_full(C.byref(guard), C.byref(decoded), now) == 0
        lib.lr_full_control_forward(forwarded, C.byref(decoded), guard.armed)
        return bytes(forwarded)

    zero = struct.pack('<BfffH',0x3c,0,0,0,0)
    assert transfer(zero,0,0,0) == zero
    assert transfer(zero,1,1,50) == zero and guard.armed
    raw = struct.pack('<BfffH',0x3c,8.123456,-6.234567,30.345678,12345)
    assert transfer(raw,1,2,100) == raw
    raw = struct.pack('<BfffH',0x30,-0.0,0.000000123,1.00000012,65535)
    assert transfer(raw,1,3,150) == raw
    # An ordinary armed command remains exact; KILL deliberately substitutes zero data.
    killed = transfer(raw,3,4,200)
    assert killed == raw[:1]+bytes(14) and not guard.armed
    assert lib.lr_guard_expire(C.byref(guard),501)
    print('PASS: actual original UDP datagrams -> bridge -> C LoRa codec -> guard -> byte-identical CRTP queue input; kill/expiry retained')
