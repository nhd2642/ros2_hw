#!/usr/bin/env python3
"""SLAM 지도 + 지금 라이다 스캔을 PNG 한 장으로 (개발용 진단, 외부 라이브러리 없음).

  python3 map_snapshot.py a1_2 /tmp/a1_2_map.png [설명]
지도 칸: 미탐색 회색 / 빈 칸 흰색 / 벽 검정. 스캔 끝점 빨강, 로봇 파랑(앞 방향 선).
같이 출력: 지도 크기·알려진 칸, 스캔 통계(유효 점 수, 거리 분포), 라이다 자세(map->lidar_link 의 roll/pitch).
지도가 작게 나오는 원인 구분용:
  - 스캔 점은 멀리 많은데 지도만 작다 -> SLAM 이 스캔을 거의 안 넣음 (minimum_travel 0.5 m / 0.5 rad, 첫 스캔만)
  - 스캔 점이 가까이(1 m 안)에만 있다 -> 라이다가 바닥을 봄(기울어짐) 또는 가림
"""
import math
import struct
import sys
import time
import zlib

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def png(path, w, h, rows):
    raw = b''.join(b'\x00' + bytes(r) for r in rows)

    def chunk(t, d):
        c = struct.pack('>I', len(d)) + t + d
        return c + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


def rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class Snap(Node):
    def __init__(self, ns):
        super().__init__('map_snapshot')
        self.ns = ns
        self.map = self.scan = None
        self.map_rx = None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(OccupancyGrid, f'/{ns}/map', self.on_map, LATCHED)
        self.create_subscription(LaserScan, f'/{ns}/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)

    def on_map(self, m):
        self.map, self.map_rx = m, time.monotonic()


def main():
    ns, out = sys.argv[1], sys.argv[2]
    label = ' '.join(sys.argv[3:])
    rclpy.init(args=['--ros-args', '-r', f'/tf:=/{ns}/tf', '-r', f'/tf_static:=/{ns}/tf_static'])  # 로봇별 TF
    n = Snap(ns)
    t0 = time.monotonic()
    tr = None
    while time.monotonic() - t0 < 15 and (n.map is None or n.scan is None or tr is None):
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.scan is not None and n.map is not None and tr is None:
            try:
                tr = n.tf.lookup_transform(n.map.header.frame_id or 'map', n.scan.header.frame_id, Time())
            except Exception:
                pass
    print(f'== map_snapshot {ns} {label}')
    if n.map is None:
        print('  지도 없음 (/%s/map 15 s 동안 못 받음)' % ns)
        return
    g = n.map
    W, H, res = g.info.width, g.info.height, g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    d = g.data
    known = sum(1 for v in d if v >= 0)
    occ = sum(1 for v in d if v >= 65)
    stamp = g.header.stamp.sec + g.header.stamp.nanosec * 1e-9
    print(f'  지도 {W}x{H} 칸 {res:.3f} m, 원점 ({ox:.2f}, {oy:.2f}), 알려진 {known} (빈 {known - occ} / 벽 {occ}), '
          f'지도 stamp {stamp:.1f}')
    pts = []
    if n.scan is None:
        print('  스캔 없음')
    else:
        s = n.scan
        good = [(i, r) for i, r in enumerate(s.ranges) if s.range_min <= r <= s.range_max]
        rs = sorted(r for _, r in good)
        if rs:
            print(f'  스캔 유효 {len(rs)}/{len(s.ranges)}, 거리 최소 {rs[0]:.2f} / 중앙 {rs[len(rs) // 2]:.2f} / '
                  f'최대 {rs[-1]:.2f} m, 1 m 안 {sum(r < 1.0 for r in rs)}점, 3 m 밖 {sum(r > 3.0 for r in rs)}점')
        else:
            print(f'  스캔 유효 0/{len(s.ranges)} (전부 inf/범위 밖)')
        near = [(math.degrees(s.angle_min + i * s.angle_increment), r) for i, r in good if r < 1.2]
        if near:                                # 이어진 방위 묶음으로 (라이다 앞 = 0°, 왼쪽 +)
            arcs, cur = [], [near[0]]
            for a, r in near[1:]:
                if a - cur[-1][0] <= 2.5 * math.degrees(s.angle_increment):
                    cur.append((a, r))
                else:
                    arcs.append(cur)
                    cur = [(a, r)]
            arcs.append(cur)
            if len(arcs) > 1 and arcs[0][0][0] - s.angle_min * 180 / math.pi < 2 and arcs[-1][-1][0] > 178:
                arcs[0] = arcs.pop() + arcs[0]     # -180/180 경계에서 이어진 묶음
            print('  1.2 m 안 점 묶음: ' + '; '.join(
                f'{len(a)}점 방위 {a[0][0]:.0f}~{a[-1][0]:.0f}° 거리 {min(r for _, r in a):.2f}~{max(r for _, r in a):.2f} m'
                for a in arcs[:6]))
        if tr is None:
            print('  TF map->라이다 없음 -> 스캔 끝점은 그리지 않음')
        else:
            p, q = tr.transform.translation, tr.transform.rotation
            roll, pitch, yaw = rpy(q)
            print(f'  라이다 자세 (map, TF 기준): ({p.x:.2f}, {p.y:.2f}, z {p.z:.2f}), roll {math.degrees(roll):.1f}°, '
                  f'pitch {math.degrees(pitch):.1f}°, yaw {math.degrees(yaw):.1f}°')
            tilt = math.hypot(roll, pitch)
            if tilt > 0.02 and p.z > 0:
                print(f'  (이 기울기면 라이다 평면이 바닥에 닿는 거리 약 {p.z / math.tan(tilt):.2f} m, TF 의 z 가 맞을 때)')
            for i, r in good:
                a = yaw + s.angle_min + i * s.angle_increment
                pts.append((p.x + r * math.cos(a), p.y + r * math.sin(a)))
            robot = (p.x, p.y, yaw)
    # 그림 (지도 행은 아래->위, PNG 는 위->아래)
    S = 3
    img = [[(128, 128, 128)] * W for _ in range(H)]
    for j in range(H):
        for i in range(W):
            v = d[j * W + i]
            if v >= 0:
                img[j][i] = (0, 0, 0) if v >= 65 else (255, 255, 255)

    def put(x, y, col):
        i, j = int((x - ox) / res), int((y - oy) / res)
        if 0 <= i < W and 0 <= j < H:
            img[j][i] = col
    for x, y in pts:
        put(x, y, (230, 30, 30))
    if pts:
        x, y, yaw = robot
        for k in range(12):
            put(x + 0.03 * k * math.cos(yaw), y + 0.03 * k * math.sin(yaw), (30, 60, 230))
        for a in range(0, 360, 20):
            put(x + 0.06 * math.cos(math.radians(a)), y + 0.06 * math.sin(math.radians(a)), (30, 60, 230))
    rows = []
    for j in range(H - 1, -1, -1):
        r = []
        for i in range(W):
            r.extend(img[j][i] * S)
        for _ in range(S):
            rows.append(r)
    png(out, W * S, H * S, rows)
    print(f'  그림: {out} (빨강 = 지금 스캔 끝점, 파랑 = 로봇)')
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
