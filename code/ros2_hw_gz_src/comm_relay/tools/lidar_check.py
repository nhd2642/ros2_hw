#!/usr/bin/env python3
"""라이다 값 점검 (2026-11-10, record17_gz_fix2 사용자 PC: 스캔 360 점이 전부 0.15 m = range_min 으로 나와 지도가 비고 로봇이 안 움직임).
/<ns>/scan 한 장과 /<ns>/terrain_camera/points 한 장을 받아 거리 분포를 출력. 스캔 점의 절반 이상이 range_min 근처면 FAIL."""
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2

rclpy.init()
n = Node('lidar_check')
scan, cloud = {}, {}
for ns in ('a1_1', 'a1_2'):
    n.create_subscription(LaserScan, f'/{ns}/scan', lambda m, ns=ns: scan.__setitem__(ns, m), qos_profile_sensor_data)
    n.create_subscription(PointCloud2, f'/{ns}/terrain_camera/points', lambda m, ns=ns: cloud.__setitem__(ns, m), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 30 and (len(scan) < 2 or len(cloud) < 2):
    rclpy.spin_once(n, timeout_sec=0.2)
ok = len(scan) == 2
for ns in ('a1_1', 'a1_2'):
    m = scan.get(ns)
    if m is None:
        print(f'{ns} 스캔: 30 s 안에 못 받음'); ok = False; continue
    r = [x for x in m.ranges if math.isfinite(x)]
    near = sum(1 for x in r if x <= m.range_min + 0.02)
    med = statistics.median(r) if r else float('nan')
    bad = (not r) or near >= 0.5 * len(r) or med < 0.3
    ok = ok and not bad
    print(f'{ns} 스캔: 유효 {len(r)}/{len(m.ranges)}, range_min {m.range_min:.2f}, 최소 {min(r, default=float("nan")):.2f} / 중앙 {med:.2f} / 최대 {max(r, default=float("nan")):.2f} m, range_min 근처 {near}점 -> {"이상" if bad else "정상"}')
for ns in ('a1_1', 'a1_2'):
    c = cloud.get(ns)
    if c is None:
        print(f'{ns} 지형 점 구름: 못 받음'); continue
    try:
        from sensor_msgs_py import point_cloud2
        xs = [float(p[0]) for p in point_cloud2.read_points(c, field_names=('x',), skip_nans=True) if math.isfinite(float(p[0]))]
        print(f'{ns} 지형 점 구름: {len(xs)}/{c.width * c.height} 점, x(앞) 최소 {min(xs, default=float("nan")):.2f} / 중앙 {statistics.median(xs) if xs else float("nan"):.2f} / 최대 {max(xs, default=float("nan")):.2f} m')
    except Exception as e:
        print(f'{ns} 지형 점 구름: {c.width * c.height} 점 (내용 읽기 실패: {e})')
print('RESULT PASS' if ok else 'RESULT FAIL')
rclpy.shutdown()
sys.exit(0 if ok else 1)
