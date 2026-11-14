#!/usr/bin/env python3
"""속도 시험: /<ns>/cmd_vel 을 주고 /<ns>/odom/ground_truth 로 측정 (CHAMP 때와 같은 지표).
  python3 rl_eval.py a1_1
시험: walk +0.3, back -0.3 (m/s, 10초), spin_L +0.5, spin_R -0.5 (rad/s, 10초). 앞 3초는 과도 구간으로 제외.
  python3 rl_eval.py a1_1 --sweep   # 명령 크기별 응답 곡선 (전진/후진 0.1~0.5 m/s, 회전 0.3~2.0 rad/s 양방향, 8초씩)
  python3 rl_eval.py a1_1 --combo   # vx=0.3 으로 걸으면서 wz=±0.2~0.8 (이름 열이 wz, 명령 열은 vx)"""
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

TESTS = [('walk', 0.3, 0.0), ('back', -0.3, 0.0), ('spin_L', 0.0, 0.5), ('spin_R', 0.0, -0.5)]
DUR, SKIP = 10.0, 3.0
if '--sweep' in sys.argv:
    DUR = 8.0
    TESTS = ([('fwd', v, 0.0) for v in (0.1, 0.2, 0.3, 0.4, 0.5)] + [('back', -v, 0.0) for v in (0.1, 0.2, 0.3, 0.4, 0.5)]
             + [('spinL', 0.0, w) for w in (0.3, 0.5, 0.8, 1.0, 1.5, 2.0)] + [('spinR', 0.0, -w) for w in (0.3, 0.5, 0.8, 1.0, 1.5, 2.0)])

if '--combo' in sys.argv:   # 걸으면서 도는 복합 명령 (vx=0.3 고정, wz 만 바꿈)
    DUR = 8.0
    TESTS = [('w%+.1f' % w, 0.3, w) for w in (0.2, 0.4, 0.6, 0.8, -0.2, -0.4, -0.6, -0.8)]


def rpy(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def main():
    ns = next((a for a in sys.argv[1:] if not a.startswith('--')), 'a1_1')
    rclpy.init()
    n = Node('rl_eval_' + ns, parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    pub = n.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
    rec = []

    def odo(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        r, pi_, y = rpy((q.x, q.y, q.z, q.w))
        rec.append((t, p.x, p.y, p.z, r, pi_, y))
    n.create_subscription(Odometry, f'/{ns}/odom/ground_truth', odo, 10)

    def now():
        return n.get_clock().now().nanoseconds * 1e-9

    def run(vx, wz, sec):
        t0 = now()
        i0 = len(rec)
        while now() - t0 < sec:
            m = Twist()
            m.linear.x, m.angular.z = float(vx), float(wz)
            pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.05)
        return i0

    t_w = now()
    while not rec and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)
    run(0.0, 0.0, 2.0)
    print(f'시작 자세: z={rec[-1][3]:.3f} roll={math.degrees(rec[-1][4]):.1f} pitch={math.degrees(rec[-1][5]):.1f} deg', flush=True)
    print(f"{'시험':<7}{'명령':>7}{'v_path':>8}{'비율':>7}{'yaw속도':>9}{'비율':>7}{'방향변화':>9}{'이동':>7}{'z':>7}{'기울기최대':>10}   (v_path m/s, yaw deg/s, 방향변화 deg, 이동 m, 기울기 deg)")
    for name, vx, wz in TESTS:
        i0 = run(vx, wz, DUR)
        R = np.array(rec[i0:], float)
        run(0.0, 0.0, 2.0)
        if len(R) < 50:
            print(f'{name}: 표본 부족 ({len(R)})')
            continue
        m = R[:, 0] >= R[0, 0] + SKIP
        W = R[m]
        yaw = np.unwrap(W[:, 6])
        ym = 0.5 * (yaw[1:] + yaw[:-1])
        T = W[-1, 0] - W[0, 0]
        vpath = float((np.cos(ym) * np.diff(W[:, 1]) + np.sin(ym) * np.diff(W[:, 2])).sum() / T)
        yr = math.degrees(yaw[-1] - yaw[0]) / T
        dist = float(math.hypot(W[-1, 1] - W[0, 1], W[-1, 2] - W[0, 2]))
        tilt = float(np.degrees(np.maximum(abs(W[:, 4]), abs(W[:, 5])).max()))
        ratio_v = vpath / vx if vx else float('nan')
        ratio_w = math.radians(yr) / wz if wz else float('nan')
        print(f"{name:<7}{(vx if vx else wz):7.2f}{vpath:8.3f}{ratio_v:7.2f}{yr:9.1f}{ratio_w:7.2f}{math.degrees(yaw[-1] - yaw[0]):9.1f}{dist:7.2f}{W[:, 3].mean():7.3f}{tilt:10.1f}", flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
