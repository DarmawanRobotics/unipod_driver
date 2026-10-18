#!/usr/bin/env python3
"""A fake UniPod MT11 on UDP for smoke tests: answers like the SDK document, pushes attitude
(0x25) and full-frame temperature (0x14 flag 2) when asked, logs every command it gets."""

import argparse
import socket
import struct
import sys
import threading
import time


def crc16(data, crc=0):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def frame(cmd, payload, seq=0):
    head = struct.pack('<HBHHB', 0x6655, 0x02, len(payload), seq, cmd) + payload
    return head + struct.pack('<H', crc16(head))


def parse(data):
    if len(data) < 10 or data[0] != 0x55 or data[1] != 0x66:
        return None
    _, _, n, seq, cmd = struct.unpack_from('<HBHHB', data)
    if len(data) < 10 + n or crc16(data[:8 + n]) != struct.unpack_from('<H', data, 8 + n)[0]:
        return None
    return cmd, seq, data[8:8 + n]


class Pod:
    RATES = {1: 2, 2: 4, 3: 5, 4: 10, 5: 20, 6: 50, 7: 100}

    def __init__(self, log):
        self.log = log
        self.zoom = (1, 0)
        self.mode = (0, 2)
        self.palette = 0
        self.gain = 0
        self.yaw = self.pitch = 0.0
        self.att_hz = 0
        self.temp_stream = False

    def reply(self, cmd, p):
        if cmd == 0x00:
            return None
        if cmd == 0x01:
            return struct.pack('<III', 0x89010001, 0x8A00010A, 0)
        if cmd == 0x05:
            return struct.pack('<H', 25)
        if cmd == 0x07:
            return b'\x01'
        if cmd == 0x08:
            self.yaw = self.pitch = 0.0
            return b'\x01'
        if cmd == 0x0D:
            return self.attitude()
        if cmd == 0x0E:
            yaw, pitch = struct.unpack('<hh', p)
            self.yaw, self.pitch = yaw / 10, pitch / 10
            return struct.pack('<hhh', yaw, pitch, 0)
        if cmd == 0x0F:
            self.zoom = (p[0], p[1])
            return b'\x01'
        if cmd == 0x10:
            return bytes(self.mode)
        if cmd == 0x11:
            self.mode = (p[0], p[1])
            return bytes(self.mode)
        if cmd == 0x12:
            x, y = struct.unpack_from('<HH', p)
            return struct.pack('<HHH', 3650, x, y)
        if cmd == 0x13:
            x1, y1, x2, y2 = struct.unpack_from('<HHHH', p)
            return struct.pack('<10H', x1, y1, x2, y2, 4510, 2133, x1, y1, x2, y2)
        if cmd == 0x14:
            self.temp_stream = p[0] == 2
            return None if p[0] != 1 else self.frame_temp()
        if cmd == 0x16:
            return bytes((30, 0))
        if cmd == 0x18:
            return bytes(self.zoom)
        if cmd == 0x1A:
            return bytes((self.palette,))
        if cmd == 0x1B:
            self.palette = p[0]
            return bytes((self.palette,))
        if cmd == 0x20:
            return struct.pack('<BBHHHB', p[0], 2, 1920, 1080, 4000, 30)
        if cmd == 0x25:
            if p[0] == 1:
                self.att_hz = self.RATES.get(p[1], 0)
            return bytes((p[0],))
        if cmd == 0x37:
            return bytes((self.gain,))
        if cmd == 0x38:
            self.gain = p[0]
            return bytes((self.gain,))
        if cmd == 0x4F:
            return b'\x01'
        return None

    def attitude(self):
        return struct.pack('<hhhhhh', int(self.yaw * 10), int(self.pitch * 10), 0, 0, 0, 0)

    @staticmethod
    def frame_temp():
        return struct.pack('<6H', 6000, 1500, 320, 256, 10, 20)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=37260)
    ap.add_argument('--log', default='-')
    a = ap.parse_args()
    out = sys.stdout if a.log == '-' else open(a.log, 'w', buffering=1)
    pod = Pod(out)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', a.port))
    peer = [None]

    def pusher():
        last_att = last_temp = 0.0
        while True:
            time.sleep(0.01)
            now = time.monotonic()
            if peer[0] is None:
                continue
            if pod.att_hz and now - last_att >= 1.0 / pod.att_hz:
                sock.sendto(frame(0x0D, pod.attitude()), peer[0])
                last_att = now
            if pod.temp_stream and now - last_temp >= 0.2:
                sock.sendto(frame(0x14, pod.frame_temp()), peer[0])
                last_temp = now

    threading.Thread(target=pusher, daemon=True).start()
    print(f'fake pod on udp 127.0.0.1:{a.port}', flush=True)
    while True:
        data, addr = sock.recvfrom(2048)
        peer[0] = addr
        f = parse(data)
        if not f:
            continue
        cmd, seq, payload = f
        print(f'cmd 0x{cmd:02X} {payload.hex()}', file=out, flush=True)
        r = pod.reply(cmd, payload)
        if r is not None:
            sock.sendto(frame(cmd, r, seq), addr)


if __name__ == '__main__':
    main()
