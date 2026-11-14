#!/usr/bin/env python3
"""B3: 최종 보행 설정에서 Nav2/복귀가 쓰는 저속 명령을 CHAMP 가 따라가는지 (gait_test 평지, a1_1).

명령마다 3 s 가속 + 6 s 측정(벽시계), 속도는 ground truth 헤더의 시뮬 시간으로 나눈다(RTF < 1 이어도 정확). /a1_1/cmd_vel 에 직접 발행
(이 시험 중에는 cmd_vel_arbiter 를 띄우지 말 것). 판정 기준(참고): 전진비 0.7~1.3, 회전비 0.7 이상, 쏠림 15° 이하.
"""
import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

CASES = [(0.05, 0.0), (0.10, 0.0), (0.15, 0.0), (0.0, 0.3), (0.0, -0.3), (0.0, 0.5), (0.0, -0.5),
         (0.15, 0.4), (0.15, -0.4)]


class N(Node):
    def __init__(self):
        super().__init__('lowspeed_check')
        self.pub = self.create_publisher(Twist, '/a1_1/cmd_vel', 10)
        self.samples = []
        self.create_subscription(Odometry, '/a1_1/odom/ground_truth', self.on, 10)

    def on(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9     # 시뮬 시간
        self.samples.append((st, p.x, p.y, yaw, p.z, time.monotonic()))

    def hold(self, vx, wz, secs):
        tw = Twist()
        tw.linear.x, tw.angular.z = vx, wz
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            self.pub.publish(tw)
            time.sleep(0.05)


def measure(s):
    fwd, dyaw = 0.0, 0.0
    for a, b in zip(s, s[1:]):
        dx, dy = b[1] - a[1], b[2] - a[2]
        fwd += dx * math.cos(a[3]) + dy * math.sin(a[3])
        dyaw += math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
    T = s[-1][0] - s[0][0]                       # 시뮬 시간
    rtf = T / max(1e-6, s[-1][5] - s[0][5])
    return fwd / T, dyaw / T, rtf


def main():
    rclpy.init()
    n = N()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(2)
    ok = True
    print(f'{"명령 vx":>8} {"wz":>6} | {"실측 v":>7} {"전진비":>6} | {"실측 wz":>7} {"회전비":>6} | {"쏠림°":>6} | 판정')
    for vx, wz in CASES:
        n.hold(vx, wz, 3.0)
        k = len(n.samples)
        n.hold(vx, wz, 6.0)
        s = n.samples[k:]
        v, w, rtf = measure(s)
        drift = math.degrees(math.atan2(math.sin(s[-1][3] - s[0][3]), math.cos(s[-1][3] - s[0][3])))
        good = True
        rv = v / vx if vx else float('nan')
        rw = w / wz if wz else float('nan')
        if vx:
            good &= 0.7 <= rv <= 1.3
        if wz:
            good &= rw >= 0.7
        if vx and not wz:
            good &= abs(drift) <= 15
        fall = min(x[4] for x in s) < 0.15
        good &= not fall
        ok &= good
        print(f'{vx:8.2f} {wz:6.2f} | {v:7.3f} {rv:6.2f} | {w:7.3f} {rw:6.2f} | {drift if vx and not wz else 0:6.1f} | '
              f'{"PASS" if good else "FAIL"}{" (넘어짐)" if fall else ""} RTF {rtf:.2f}', flush=True)
        n.hold(0.0, 0.0, 2.0)
    print(f'RESULT {"PASS" if ok else "FAIL"}')
    sys.stdout.flush()
    os._exit(0 if ok else 1)   # rclpy 스레드 정리 중 'terminate called' Aborted 방지 (결과는 이미 출력됨)


if __name__ == '__main__':
    main()
