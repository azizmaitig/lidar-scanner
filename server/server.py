import asyncio
import struct
import json
import argparse
import os
from collections import deque

from aiohttp import web


class LidarReader:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.reader = None
        self.writer = None
        self._buf = bytearray()
        self._last_counter = None
        self._rotation_count = 0
        self.packets_total = 0
        self.packets_errors = 0

    async def connect(self):
        self.reader, self.writer = await asyncio.open_connection(
            self.host, self.port
        )
        print(f"LiDAR connected to {self.host}:{self.port}")

    async def read_packets(self):
        while True:
            try:
                chunk = await self.reader.read(1024)
                if not chunk:
                    print("LiDAR connection closed")
                    await asyncio.sleep(1)
                    await self.connect()
                    continue
                self._buf.extend(chunk)
                pkts = self._extract_packets()
                for pkt in pkts:
                    yield pkt
            except asyncio.CancelledError:
                return
            except Exception as e:
                print(f"LiDAR read error: {e}")
                await asyncio.sleep(1)
                try:
                    await self.connect()
                except Exception:
                    pass

    def _extract_packets(self):
        pkts = []
        while True:
            idx = self._buf.find(b'\x55')
            if idx < 0:
                self._buf.clear()
                break
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < 36:
                break
            candidate = bytes(self._buf[:36])
            if candidate[0] == 0x55:
                pkts.append(candidate)
                del self._buf[:36]
            else:
                del self._buf[:1]
        return pkts

    def parse_packet(self, raw):
        if len(raw) != 36 or raw[0] != 0x55:
            return None
        counter = struct.unpack('<H', raw[6:8])[0]
        samples = []
        for i in range(8):
            off = 8 + i * 3
            q = raw[off]
            d_raw = struct.unpack('>H', raw[off+1:off+3])[0]
            d_mm = d_raw * 0.25 if d_raw < 0x8000 else 0
            samples.append({'quality': q, 'distance_mm': round(d_mm, 1)})
        return {'counter': counter, 'samples': samples}


class EncoderReader:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.reader = None
        self.writer = None
        self._buf = bytearray()

    async def connect(self):
        self.reader, self.writer = await asyncio.open_connection(
            self.host, self.port
        )
        print(f"Encoder connected to {self.host}:{self.port}")

    def _parse_packet(self):
        if len(self._buf) < 14:
            return None
        if self._buf[0] != 0xAA or self._buf[1] != 0x01:
            del self._buf[:1]
            return None
        ts = struct.unpack_from('>I', self._buf, 2)[0]
        pos = struct.unpack_from('>i', self._buf, 6)[0]
        speed = struct.unpack_from('>f', self._buf, 10)[0]
        del self._buf[:14]
        return {'timestamp': ts, 'position': pos, 'speed': speed}

    async def read_encoder(self):
        while True:
            try:
                chunk = await self.reader.read(256)
                if not chunk:
                    await asyncio.sleep(0.1)
                    continue
                self._buf.extend(chunk)
                while True:
                    pkt = self._parse_packet()
                    if pkt is None:
                        break
                    yield pkt
            except asyncio.CancelledError:
                return
            except Exception as e:
                print(f"Encoder read error: {e}")
                await asyncio.sleep(1)
                try:
                    await self.connect()
                except Exception:
                    pass


class LidarServer:
    def __init__(self, lidar_host='192.168.1.79', lidar_port=23,
                 http_port=8765, encoder_host=None, encoder_port=0,
                 lidar_disable=False):
        self.lidar_host = lidar_host
        self.lidar_port = lidar_port
        self.http_port = http_port
        self.encoder_host = encoder_host
        self.encoder_port = encoder_port
        self.lidar_disable = lidar_disable
        self.reader = LidarReader(lidar_host, lidar_port) if not lidar_disable else None
        self.encoder = None
        if encoder_host:
            self.encoder = EncoderReader(encoder_host, encoder_port)
        self.ws_clients = set()
        self._latest_scan = None
        self._rotation_count = 0
        self._tilt_angle = 0.0

    async def ws_handler(self, request):
        ws = web.WebSocketResponse(max_msg_size=0)
        await ws.prepare(request)
        self.ws_clients.add(ws)
        print(f"WS client connected ({len(self.ws_clients)} total)")
        if self._latest_scan:
            await ws.send_str(json.dumps(self._latest_scan))
        try:
            async for msg in ws:
                pass
        except asyncio.CancelledError:
            pass
        finally:
            self.ws_clients.discard(ws)
        return ws

    async def broadcast(self, data):
        self._latest_scan = data
        msg = json.dumps(data)
        dead = set()
        for ws in self.ws_clients:
            try:
                await ws.send_str(msg)
            except Exception:
                dead.add(ws)
        self.ws_clients -= dead

    def rotation_to_3d(self, samples, tilt_angle_deg, total_samples):
        tilt_rad = tilt_angle_deg * (3.14159 / 180.0)
        ct = __import__('math').cos(tilt_rad)
        st = __import__('math').sin(tilt_rad)
        points = []
        for idx, s in enumerate(samples):
            if s['distance_mm'] <= 0:
                continue
            theta = (idx / total_samples) * 360.0
            theta_rad = theta * (3.14159 / 180.0)
            r = s['distance_mm']
            x = r * __import__('math').cos(theta_rad) * ct
            y = r * __import__('math').sin(theta_rad) * ct
            z = r * st
            points.append({
                'x': round(x, 1),
                'y': round(y, 1),
                'z': round(z, 1),
                'quality': s['quality'],
            })
        return points

    async def process_stream(self):
        await self.reader.connect()
        last_counter = None
        rotation_samples = []
        rotation_packets = 0

        async for raw in self.reader.read_packets():
            pkt = self.reader.parse_packet(raw)
            if pkt is None:
                continue
            counter = pkt['counter']
            is_new_rotation = (
                last_counter is not None and
                last_counter - counter > 1000
            )

            if is_new_rotation and rotation_samples:
                self._rotation_count += 1
                total = len(rotation_samples)
                dists = [s['distance_mm'] for s in rotation_samples if s['distance_mm'] > 0]

                points_3d = self.rotation_to_3d(
                    rotation_samples, self._tilt_angle, total
                )

                payload = {
                    'type': 'scan',
                    'rotation': self._rotation_count,
                    'packets': rotation_packets,
                    'samples': total,
                    'valid': len(dists),
                    'min_dist': round(min(dists)) if dists else 0,
                    'max_dist': round(max(dists)) if dists else 0,
                    'tilt_angle': round(self._tilt_angle, 2),
                    'points': points_3d,
                }
                await self.broadcast(payload)
                rotation_samples = []
                rotation_packets = 0
                await asyncio.sleep(0)

            for s in pkt['samples']:
                rotation_samples.append(s)
            rotation_packets += 1
            last_counter = counter

    async def process_encoder(self):
        async for enc in self.encoder.read_encoder():
            angle_deg = (enc['position'] / 400) * 360.0
            self._tilt_angle = angle_deg

    async def handle_index(self, request):
        return web.FileResponse(
            os.path.join(os.path.dirname(__file__), 'viewer.html')
        )

    async def start(self):
        app = web.Application()
        app.router.add_get('/', self.handle_index)
        app.router.add_get('/ws', self.ws_handler)

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '0.0.0.0', self.http_port)
        await site.start()
        print(f"Server at http://localhost:{self.http_port}")

        tasks = []
        if not self.lidar_disable:
            tasks.append(asyncio.create_task(self.process_stream()))
        if self.encoder:
            tasks.append(asyncio.create_task(self.process_encoder()))
        if tasks:
            await asyncio.gather(*tasks)
        else:
            await asyncio.Event().wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='192.168.1.79')
    parser.add_argument('--port', type=int, default=23)
    parser.add_argument('--http-port', type=int, default=8765)
    parser.add_argument('--encoder-host',
                        help='ESP8266 encoder TCP host')
    parser.add_argument('--encoder-port', type=int, default=0,
                        help='ESP8266 encoder TCP port')
    parser.add_argument('--lidar-disable', action='store_true',
                        help='Skip LiDAR connection (test viewer without ESP)')
    args = parser.parse_args()

    server = LidarServer(
        lidar_host=args.host,
        lidar_port=args.port,
        http_port=args.http_port,
        encoder_host=args.encoder_host,
        encoder_port=args.encoder_port or 0,
        lidar_disable=args.lidar_disable,
    )

    try:
        asyncio.run(server.start())
    except KeyboardInterrupt:
        print("\nShutdown")
