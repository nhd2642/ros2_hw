#!/usr/bin/env python3
"""SLAM 이 스캔을 지도에 넣는지 진단 (개발용). comm_relay 없이 sim + dual_slam 만 떠 있을 때 쓴다.

  python3 slam_diag.py a1_2
1) 10 s 관찰: /ns/scan, /ns/scan_ready 수, /ns/tf 의 (부모->자식) 쌍과 빈도, 스캔 stamp 와 그때까지 받은
   odom->base_footprint TF stamp 의 차이(TF 가 스캔보다 늦으면 transform_timeout 0 인 slam_toolbox 가 버림),
   base_link 의 roll/pitch(라이다 기울기), 지도 알려진 칸
2) 제자리 360° 회전 (CHAMP /ns/cmd_vel 직접, 끝나면 0 명령) -> 12 s 기다려 지도 칸 (Humble slam_toolbox 는 회전만으로는 안 늚)
3) 1.0 m 전진 -> 12 s 기다려 지도 칸
slam_toolbox(Humble) 는 직선 이동이 sqrt(0.8) x minimum_travel_distance(0.5) = 약 0.45 m 넘을 때만 스캔을 넣는다
(minimum_travel_heading 은 이 단계에서 안 봄). 그래서 3) 에서 늘면 정상.
시간은 /clock (시뮬 시간) 기준으로도 함께 출력.
"""
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def st(s):
    return s.sec + s.nanosec * 1e-9


def rp(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return math.degrees(roll), math.degrees(pitch)


class D(Node):
    def __init__(self, ns):
        super().__init__('slam_diag')
        self.ns = ns
        self.n_scan = self.n_ready = 0
        self.pairs = {}
        self.last_odom_tf = None            # odom->base_footprint 의 최신 stamp
        self.lags = []                      # 스캔 stamp - 그때 최신 odom TF stamp
        self.ready_lags = []
        self.base_rp = []
        self.map = None
        self.map_hist = []
        self.clock = None
        self.gt = None
        self.ranges_min = []
        self.create_subscription(LaserScan, f'/{ns}/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, f'/{ns}/scan_ready', self.on_ready, qos_profile_sensor_data)
        self.create_subscription(TFMessage, f'/{ns}/tf', self.on_tf, 100)
        self.create_subscription(TFMessage, f'/{ns}/tf_static', self.on_tf, LATCHED)
        self.create_subscription(OccupancyGrid, f'/{ns}/map', self.on_map, LATCHED)
        self.create_subscription(Clock, '/clock', lambda m: setattr(self, 'clock', st(m.clock)), qos_profile_sensor_data)
        self.create_subscription(Odometry, f'/{ns}/odom/ground_truth', self.on_gt, 10)
        self.pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)

    def on_gt(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.gt = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_scan(self, m):
        self.n_scan += 1
        if self.last_odom_tf is not None:
            self.lags.append(st(m.header.stamp) - self.last_odom_tf)
        good = [r for r in m.ranges if m.range_min <= r <= m.range_max]
        if good:
            self.ranges_min.append(min(good))

    def on_ready(self, m):
        self.n_ready += 1
        if self.last_odom_tf is not None:
            self.ready_lags.append(st(m.header.stamp) - self.last_odom_tf)

    def on_tf(self, m):
        for t in m.transforms:
            k = (t.header.frame_id, t.child_frame_id)
            self.pairs[k] = self.pairs.get(k, 0) + 1
            if t.child_frame_id.endswith('base_footprint') and t.header.frame_id.endswith('odom'):
                self.last_odom_tf = st(t.header.stamp)
            if t.child_frame_id.endswith('base_link') and t.header.frame_id.endswith('base_footprint'):
                self.base_rp.append(rp(t.transform.rotation) + (t.transform.translation.z,))

    def on_map(self, m):
        self.map = m
        self.map_hist.append((time.monotonic(), st(m.header.stamp), sum(1 for v in m.data if v >= 0)))

    def known(self):
        return self.map_hist[-1][2] if self.map_hist else None

    def cmd(self, vx, wz):
        t = Twist()
        t.linear.x, t.angular.z = vx, wz
        self.pub.publish(t)


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))] if v else float('nan')


def main():
    ns = sys.argv[1] if len(sys.argv) > 1 else 'a1_2'
    rclpy.init()
    d = D(ns)
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(d)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    print(f'== slam_diag {ns}', flush=True)
    time.sleep(2.0)
    c0, w0 = d.clock, time.monotonic()
    d.n_scan = d.n_ready = 0
    d.pairs, d.lags, d.ready_lags, d.base_rp, d.ranges_min = {}, [], [], [], []
    time.sleep(10.0)
    dt_sim = (d.clock - c0) if (d.clock and c0) else float('nan')
    print(f'1) 10 s 관찰 (시뮬 {dt_sim:.1f} s, RTF {dt_sim / (time.monotonic() - w0):.2f})')
    print(f'   /{ns}/scan {d.n_scan}개, /{ns}/scan_ready {d.n_ready}개 (relay 가 TF 를 기다리다 버리면 ready 가 적다)')
    for (a, b), n in sorted(d.pairs.items()):
        print(f'   TF {a} -> {b}: {n}개')
    if d.lags:
        print(f'   스캔 stamp - 최신 odom TF stamp: 중앙 {pct(d.lags, 0.5) * 1000:.0f} ms, 90% {pct(d.lags, 0.9) * 1000:.0f} ms, '
              f'최대 {max(d.lags) * 1000:.0f} ms (양수 = 스캔 시각의 TF 가 아직 없음)')
    else:
        print('   odom -> base_footprint TF 없음 (EKF TF 이름 확인)')
    if d.ready_lags:
        print(f'   scan_ready stamp - 최신 odom TF: 중앙 {pct(d.ready_lags, 0.5) * 1000:.0f} ms, 최대 {max(d.ready_lags) * 1000:.0f} ms')
    if d.base_rp:
        rs = [r for r, _, _ in d.base_rp]
        ps = [p for _, p, _ in d.base_rp]
        zs = [z for _, _, z in d.base_rp]
        print(f'   base_footprint->base_link: roll {min(rs):.1f}~{max(rs):.1f}°, pitch {min(ps):.1f}~{max(ps):.1f}°, '
              f'z {min(zs):.3f}~{max(zs):.3f} m')
    if d.ranges_min:
        print(f'   스캔 최소 거리: 중앙 {pct(d.ranges_min, 0.5):.2f} m, 최소 {min(d.ranges_min):.2f} m')
    print(f'   지도 알려진 칸 {d.known()} (지도 수신 {len(d.map_hist)}회)', flush=True)
    k0 = d.known()

    def wait_gt():
        t = time.monotonic()
        while d.gt is None and time.monotonic() - t < 10:
            time.sleep(0.1)
        return d.gt is not None

    if not wait_gt():
        print('   ground truth 없음 -> 움직임 시험 생략')
        return
    # 2) 360° 회전
    prev, turned, t = d.gt[2], 0.0, time.monotonic()
    while turned < 2 * math.pi and time.monotonic() - t < 120:
        d.cmd(0.0, 0.4)
        time.sleep(0.05)
        y = d.gt[2]
        turned += abs(math.atan2(math.sin(y - prev), math.cos(y - prev)))
        prev = y
    for _ in range(20):
        d.cmd(0.0, 0.0)
        time.sleep(0.05)
    time.sleep(12.0)
    k1 = d.known()
    print(f'2) 제자리 {math.degrees(turned):.0f}° 회전 ({time.monotonic() - t:.0f} s) 뒤 지도 알려진 칸 {k0} -> {k1}', flush=True)
    # 3) 1 m 전진
    x0, y0, _ = d.gt
    t = time.monotonic()
    while math.hypot(d.gt[0] - x0, d.gt[1] - y0) < 1.0 and time.monotonic() - t < 60:
        d.cmd(0.15, 0.0)
        time.sleep(0.05)
    for _ in range(20):
        d.cmd(0.0, 0.0)
        time.sleep(0.05)
    moved = math.hypot(d.gt[0] - x0, d.gt[1] - y0)
    time.sleep(12.0)
    k2 = d.known()
    print(f'3) {moved:.2f} m 전진 뒤 지도 알려진 칸 {k1} -> {k2}', flush=True)
    grew = k0 is not None and k2 is not None and k2 > k0 * 1.3
    print('RESULT PASS SLAM 이 움직임에 따라 지도를 키움' if grew else
          'RESULT FAIL SLAM 지도가 안 늘어남 (아래 slam 로그 경고 확인)', flush=True)
    print('   (참고: slam_toolbox Humble 은 직선 이동 약 0.45 m 마다 스캔을 넣고, 제자리 회전은 세지 않는다)', flush=True)
    ex.shutdown()
    d.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
