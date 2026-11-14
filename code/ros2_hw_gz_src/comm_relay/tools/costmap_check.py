#!/usr/bin/env python3
"""국소·전역 비용지도에 라이다 장애물이 실제로 찍히는지 확인 (로봇은 스폰에 서 있음, 탐색 없음).

왜: record5~9 표본 약 17,000 개 중 '국소 비용지도 0.45 m 안 치명 칸'이 지형 위험 없이 나온 경우가 13 번뿐.
  a1_2 가 Husky 모서리 0.22 m·벽 끝 0.22 m 사이에서 멈췄을 때도 0 칸 (2026-11-09 record9). 라이다 층이 로봇 근처를 못 찍는지 확인.
출력 (로봇마다): 스캔 Hz, 3 m 안 스캔 끝점 수, 그 끝점 칸 중 국소 비용지도 >= 99 비율, 국소 창 전체 치명(100) 칸 수, 전역 치명 칸 수.
끝점은 /<ns>/odom 자세 + 라이다가 몸 중심 위라고 보고 odom 으로 옮김 (라이다 x, y 오프셋 약 2 cm — 5 cm 칸이라 무시).
"""
import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

ROBOTS = ('a1_1', 'a1_2')
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def cell(g, x, y):
    i = int((x - g.info.origin.position.x) / g.info.resolution)
    j = int((y - g.info.origin.position.y) / g.info.resolution)
    if 0 <= i < g.info.width and 0 <= j < g.info.height:
        return g.data[j * g.info.width + i]
    return None


class Check(Node):
    def __init__(self):
        super().__init__('costmap_check')
        self.d = {ns: dict(scan=None, n_scan=0, odom=None, cm=None, gm=None, hits=[0, 0]) for ns in ROBOTS}
        for ns in ROBOTS:
            self.create_subscription(LaserScan, f'/{ns}/scan', lambda m, n=ns: self.on_scan(n, m), qos_profile_sensor_data)
            self.create_subscription(Odometry, f'/{ns}/odom', lambda m, n=ns: self.d[n].__setitem__('odom', m), 10)
            for key, topic in (('cm', 'local_costmap/costmap'), ('gm', 'global_costmap/costmap')):
                for q in (LATCHED, 10):
                    self.create_subscription(OccupancyGrid, f'/{ns}/{topic}', lambda m, n=ns, k=key: self.d[n].__setitem__(k, m), q)

    def on_scan(self, ns, m):
        d = self.d[ns]
        d['scan'], d['n_scan'] = m, d['n_scan'] + 1
        g, o = d['cm'], d['odom']
        if g is None or o is None:
            return
        p = o.pose.pose
        yaw = yaw_of(p.orientation)
        for k, r in enumerate(m.ranges):
            if not (math.isfinite(r) and 0.25 <= r <= 1.4):       # 3x3 창 안에 들어오는 거리
                continue
            a = yaw + m.angle_min + k * m.angle_increment
            c = cell(g, p.position.x + r * math.cos(a), p.position.y + r * math.sin(a))
            if c is None:
                continue
            d['hits'][1] += 1
            d['hits'][0] += c >= 99


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    n = Check()
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.1)
    el = time.monotonic() - t0
    for ns in ROBOTS:
        d = n.d[ns]
        cm, gm = d['cm'], d['gm']
        lc = int((np.asarray(cm.data) == 100).sum()) if cm else None
        gc = int((np.asarray(gm.data) == 100).sum()) if gm else None
        hit = d['hits']
        frac = f'{hit[0] / hit[1]:.0%}' if hit[1] else '-'
        print(f'{ns}: 스캔 {d["n_scan"] / el:.1f} Hz, 0.25~1.4 m 끝점 {hit[1]} 개 중 국소 비용지도 >=99 칸 {frac} | '
              f'국소 창 치명 칸 {lc} ({"없음" if cm is None else f"{cm.info.width}x{cm.info.height}, frame {cm.header.frame_id}"}), '
              f'전역 치명 칸 {gc}', flush=True)
    ok = all(n.d[ns]['hits'][1] and n.d[ns]['hits'][0] / n.d[ns]['hits'][1] >= 0.5 for ns in ROBOTS)
    print('RESULT', 'PASS (라이다 끝점이 국소 비용지도에 찍힘)' if ok else 'FAIL (라이다 끝점 대부분이 국소 비용지도에 없음)')
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
