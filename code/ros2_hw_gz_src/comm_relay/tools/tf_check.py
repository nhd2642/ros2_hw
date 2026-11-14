#!/usr/bin/env python3
"""국소 비용지도 라이다 층이 하는 TF 변환을 그대로 해 봄: 스캔마다 스캔 시각의 odom <- 스캔 프레임.

왜: cmcheck (2026-11-09) 에서 라이다 끝점 약 2~3 만 개 중 국소 비용지도에 찍힌 것 0 % (두 로봇, 지형 층 켬/끔), 로그에 오류 없음.
  ObstacleLayer 는 tf2 MessageFilter(대기 = transform_tolerance) 로 스캔 시각 TF 를 기다리고, 못 구하면 조용히 버림.
출력 (로봇마다): 스캔 수, 바로 변환 성공 / 0.6 s 기다린 뒤 성공 / 실패 (이유별), TF 링크별로 스캔 시각 대비 최신 TF 시각 차 (중앙값).
"""
import sys
import time
from collections import Counter

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

ROBOTS = ('a1_1', 'a1_2')
WAIT = 0.6        # local costmap transform_tolerance (팀원 설정)


class Robot(Node):
    def __init__(self, ns):
        super().__init__(f'tf_check_{ns.replace("_", "")}',
                         cli_args=['--ros-args', '-r', f'/tf:=/{ns}/tf', '-r', f'/tf_static:=/{ns}/tf_static'])
        self.ns = ns
        self.buf = Buffer(cache_time=Duration(seconds=10))
        self.tl = TransformListener(self.buf, self)
        self.pending = []                     # (스캔 시각 Time, frame, 받은 벽시계)
        self.res = Counter()
        self.err = Counter()
        self.edges = {}                       # (parent, child) -> 최신 stamp (s)
        self.lags = {}                        # (parent, child) -> [스캔 시각 - 최신 TF 시각]
        self.frame = None
        self.create_subscription(LaserScan, f'/{ns}/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(TFMessage, f'/{ns}/tf', self.on_tf, 100)
        self.create_timer(0.05, self.retry)

    def on_tf(self, m):
        for t in m.transforms:
            self.edges[(t.header.frame_id, t.child_frame_id)] = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9

    def on_scan(self, m):
        st = Time.from_msg(m.header.stamp)
        self.frame = m.header.frame_id
        s = st.nanoseconds * 1e-9
        for e, ts in self.edges.items():
            if e[0] in ('odom', 'base_footprint', 'base_link') or e[1] in ('base_footprint', 'base_link'):
                self.lags.setdefault(e, []).append(s - ts)
        try:
            self.buf.lookup_transform('odom', m.header.frame_id, st)
            self.res['바로 성공'] += 1
        except Exception:
            self.pending.append((st, m.header.frame_id, time.monotonic()))

    def retry(self):
        keep = []
        for st, fr, t0 in self.pending:
            try:
                self.buf.lookup_transform('odom', fr, st)
                self.res[f'{WAIT} s 안에 성공'] += 1
            except Exception as e:
                if time.monotonic() - t0 < WAIT:
                    keep.append((st, fr, t0))
                else:
                    self.res['실패'] += 1
                    msg = str(e).split('.')[0][:110]
                    self.err[msg] += 1
        self.pending = keep


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    nodes = [Robot(ns) for ns in ROBOTS]
    ex = MultiThreadedExecutor()
    for n in nodes:
        ex.add_node(n)
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        ex.spin_once(timeout_sec=0.1)
    ok_all = True
    for n in nodes:
        tot = sum(n.res.values())
        print(f'{n.ns}: 스캔 {tot} 개 (프레임 {n.frame}) -> ' + ', '.join(f'{k} {v}' for k, v in n.res.most_common()), flush=True)
        for k, v in n.err.most_common(3):
            print(f'    실패 이유 x{v}: {k}')
        for e, l in sorted(n.lags.items()):
            l = sorted(l)
            print(f'    TF {e[0]} -> {e[1]}: 스캔 시각 - 최신 TF 시각 중앙 {l[len(l) // 2]:+.3f} s (최소 {l[0]:+.3f}, 최대 {l[-1]:+.3f})')
        ok_all &= tot > 0 and n.res['실패'] <= 0.2 * tot
    print('RESULT', 'PASS (스캔 시각 TF 를 구함)' if ok_all else 'FAIL (스캔 시각 TF 를 못 구하는 스캔이 많음)')
    for n in nodes:
        n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
