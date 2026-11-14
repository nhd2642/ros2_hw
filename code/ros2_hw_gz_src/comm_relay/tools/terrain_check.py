#!/usr/bin/env python3
"""깊이 카메라 지형 위험 확인 (Gazebo, 개발용). sim + SLAM + Nav2 + terrain_hazards 가 떠 있을 때.

  python3 terrain_check.py
a1_1 (스폰 (0, -1.1), 남쪽을 봄) 남쪽에 ramp_south_a/b (x +-0.95, y -1.99 ~ -3.98, 10° 오르막 뒤 내리막).
1) 깊이 점구름이 오는지, 프레임이 맞는지: odom 으로 옮긴 앞 바닥 점의 높이 중앙값 |z| < 0.03 m
2) 평지 (0, -1.75) 까지 남쪽으로 간 뒤 그 자리에서 남서쪽으로 돌고, 경사로를 비스듬히 올라 (-0.6, -2.2) 에서 멈춤
   (cmd_vel 직접, 끝나면 0). 넘어졌던 상황(경사로 위에서 옆 가장자리 쪽으로 감)과 같은 시점: 서쪽 가장자리가
   약 0.35 m, 시야 안(정면에서 약 37°). 경사로 위 제자리 회전은 하지 않음 (1차: 회전 중 0.6 m 미끄러짐).
   경사로 가운데 정면에서는 옆 가장자리 바깥 바닥이 시야·그늘에 들어 안 보이는 것이 정상 (1차 시험의 잘못된 기대).
3) /a1_1/terrain_hazards: 오르는 동안 내내 경사로 표면(|x| < 0.6)에는 위험 없음, 서쪽 옆 가장자리(x -1.05 ~ -0.85)에 위험 있음
4) 국소·전역 비용지도: 서쪽 옆 가장자리에 치명 칸 있음 (가운데는 경사로 위에서 몸이 기울면 라이다가 바닥을 찍을 수
   있어 비용지도로는 판정하지 않고 3) 의 지형 위험으로만 봄)
RTF, 몸통 pitch 도 출력. 좌표: a1_1 odom/map = 월드 방향, 원점 스폰 -> 월드 = odom + (0, -1.1).
"""
import math
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

from terrain_perception.terrain_node import cloud_xyz, tf_matrix

NS, SPAWN = 'a1_1', (0.0, -1.1)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
STATIC = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
EDGE = lambda x, y: (x >= -1.05) & (x <= -0.85) & (y <= -2.05) & (y >= -2.95)   # 서쪽 옆 가장자리   # noqa: E731
CENTER = lambda x, y: (np.abs(x) < 0.6) & (y <= -2.05) & (y >= -3.4)              # 경사로 표면                          # noqa: E731


class Chk(Node):
    def __init__(self):
        super().__init__('terrain_check')
        self.buf = Buffer(cache_time=Duration(seconds=10.0))
        self.cloud = self.hz = self.fr = None
        self.n_cloud = 0
        self.cm = {}
        self.gt = None
        self.rp = (0.0, 0.0)
        self.sim = None
        self.create_subscription(TFMessage, f'/{NS}/tf', lambda m: [self.buf.set_transform(t, 'c') for t in m.transforms], 100)
        self.create_subscription(TFMessage, f'/{NS}/tf_static',
                                 lambda m: [self.buf.set_transform_static(t, 'c') for t in m.transforms], STATIC)
        self.create_subscription(PointCloud2, f'/{NS}/terrain_camera/points', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, f'/{NS}/terrain_hazards', lambda m: setattr(self, 'hz', cloud_xyz(m)), 5)
        self.create_subscription(PointCloud2, f'/{NS}/terrain_free', lambda m: setattr(self, 'fr', cloud_xyz(m)), 5)
        for k in ('local_costmap', 'global_costmap'):
            self.create_subscription(OccupancyGrid, f'/{NS}/{k}/costmap', lambda m, k=k: self.cm.__setitem__(k, m), LATCHED)
        self.create_subscription(Odometry, f'/{NS}/odom/ground_truth', self.on_gt, 10)
        self.create_subscription(Clock, '/clock', lambda m: setattr(self, 'sim', m.clock.sec + m.clock.nanosec * 1e-9),
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, f'/{NS}/cmd_vel', 10)

    def on_cloud(self, m):
        self.cloud = m
        self.n_cloud += 1

    def on_gt(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.gt = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        self.rp = (math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))),
                   math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))))


def cost_cells(g, pred):
    """OccupancyGrid -> pred(월드 x, y) 안 칸 값 배열."""
    w, h, r = g.info.width, g.info.height, g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    d = np.asarray(g.data, dtype=np.int16).reshape(h, w)
    jj, ii = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
    x = ox + (ii + 0.5) * r + SPAWN[0]
    y = oy + (jj + 0.5) * r + SPAWN[1]
    return d[pred(x, y)]


def main():
    rclpy.init()
    n = Chk()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    res = []

    def add(name, ok, detail):
        res.append(ok)
        print(f'  [{"PASS" if ok else "FAIL"}] {name}: {detail}', flush=True)

    print('== terrain_check', flush=True)
    t0, s0 = time.monotonic(), None
    while time.monotonic() - t0 < 20 and (n.cloud is None or n.gt is None):
        time.sleep(0.2)
    s0, w0 = n.sim, time.monotonic()
    c0 = n.n_cloud
    time.sleep(5.0)
    rtf = (n.sim - s0) / (time.monotonic() - w0) if (n.sim and s0) else float('nan')
    hzc = (n.n_cloud - c0) / 5.0
    if n.cloud is None:
        add('깊이 점구름 /a1_1/terrain_camera/points', False, '없음 (URDF terrain_camera / 토픽 이름 확인: ros2 topic list | grep terrain)')
        print('RESULT FAIL')
        sys.stdout.flush()
        import os
        os._exit(1)
    c = n.cloud
    add('깊이 점구름 수신', hzc > 2, f'{hzc:.1f} Hz (벽시계), {c.width}x{c.height}, frame {c.header.frame_id}, RTF {rtf:.2f}')
    # 1) 프레임 확인
    try:
        st = Time.from_msg(c.header.stamp)
        deadline = time.monotonic() + 3.0
        while not n.buf.can_transform('odom', c.header.frame_id, st) and time.monotonic() < deadline:
            time.sleep(0.1)
        R, t = tf_matrix(n.buf.lookup_transform('odom', c.header.frame_id, st))
        Rb, tb = tf_matrix(n.buf.lookup_transform('odom', 'base_footprint', st))
        p = cloud_xyz(c)
        po = p @ R.T + t
        rel = po[:, :2] - tb[:2]
        yaw = math.atan2(Rb[1, 0], Rb[0, 0])
        fwd = rel[:, 0] * math.cos(yaw) + rel[:, 1] * math.sin(yaw)
        side = -rel[:, 0] * math.sin(yaw) + rel[:, 1] * math.cos(yaw)
        sel = (fwd > 0.3) & (fwd < 0.8) & (np.abs(side) < 0.4)
        zmed = float(np.median(po[sel, 2])) if sel.any() else float('nan')
        add('점구름 프레임: 로봇 앞 0.3~0.8 m 바닥 높이 중앙값 |z| < 0.03 m', sel.sum() > 50 and abs(zmed) < 0.03,
            f'{zmed:+.3f} m ({int(sel.sum())}점). 크게 틀리면 Gazebo 점구름이 광학 프레임이 아님 -> cloud_frame:=terrain_camera_link')
    except Exception as e:
        add('점구름 프레임 확인 (TF)', False, f'{type(e).__name__}: {e}')
    # 2) 경사로에 올라 서쪽을 봄. 그동안 경사로 표면 위험 최대값 기록
    cen_max = [0]

    def watch():
        hz = n.hz if n.hz is not None else np.zeros((0, 3))
        cen_max[0] = max(cen_max[0], int(CENTER(hz[:, 0] + SPAWN[0], hz[:, 1] + SPAWN[1]).sum()))

    def go(tx, ty, tol=0.08, limit=90):
        t1 = time.monotonic()
        while time.monotonic() - t1 < limit:
            x, y, yaw = n.gt
            if math.hypot(tx - x, ty - y) < tol:
                return True
            err = math.atan2(math.sin(math.atan2(ty - y, tx - x) - yaw), math.cos(math.atan2(ty - y, tx - x) - yaw))
            tw = Twist()
            tw.linear.x, tw.angular.z = (0.0, math.copysign(0.4, err)) if abs(err) > 0.4 else (0.12, err)
            n.pub.publish(tw)
            watch()
            time.sleep(0.05)
        return False

    def turn(target, limit=60):
        t1 = time.monotonic()
        while time.monotonic() - t1 < limit:
            err = math.atan2(math.sin(target - n.gt[2]), math.cos(target - n.gt[2]))
            if abs(err) < 0.08:
                return True
            tw = Twist()
            tw.angular.z = math.copysign(0.4, err)
            n.pub.publish(tw)
            watch()
            time.sleep(0.05)
        return False

    TX, TY = -0.6, -2.2
    ok_path = go(0.0, -1.75) and turn(math.atan2(TY + 1.75, TX)) and go(TX, TY)
    for _ in range(20):
        n.pub.publish(Twist())
        time.sleep(0.05)
    time.sleep(1.0)
    err = math.hypot(n.gt[0] - TX, n.gt[1] - TY)
    add(f'경사로 위 ({TX}, {TY}) 에서 남서쪽(서쪽 가장자리 0.35 m)을 보고 멈춤', ok_path and err < 0.15,
        f'월드 ({n.gt[0]:.2f}, {n.gt[1]:.2f}), 목표와 {err:.2f} m, yaw {math.degrees(n.gt[2]):.0f}°, '
        f'몸 roll/pitch {n.rp[0]:+.1f}/{n.rp[1]:+.1f}° (GT)')
    t1 = time.monotonic()
    while time.monotonic() - t1 < 5.0:
        watch()
        time.sleep(0.1)
    # 3) 위험 점
    hz = n.hz if n.hz is not None else np.zeros((0, 3))
    hx, hy = hz[:, 0] + SPAWN[0], hz[:, 1] + SPAWN[1]
    e_w = int(EDGE(hx, hy).sum())
    nfr = 0 if n.fr is None else len(n.fr)
    add('지형 위험: 오르는 동안 경사로 표면(10°)에는 없음 (최대)', cen_max[0] == 0, f'최대 {cen_max[0]}칸')
    add('지형 위험: 서쪽 옆 가장자리에 있음 (3칸 이상)', e_w >= 3,
        f'{e_w}칸 (전체 위험 {len(hz)}, 이번 빈 칸 {nfr})'
        + ('' if e_w >= 3 or not len(hz) else f', 위험 예 {np.round(np.stack([hx, hy], 1)[:6], 2).tolist()}'))
    # 4) 비용지도
    for k in ('local_costmap', 'global_costmap'):
        g = n.cm.get(k)
        if g is None:
            add(f'{k} 수신', False, '없음')
            continue
        edge = cost_cells(g, EDGE)
        add(f'{k}: 서쪽 옆 가장자리 치명 칸 있음', (edge >= 99).sum() >= 3,
            f'치명 {(edge >= 99).sum()}/{len(edge)}, 미탐색 {(edge < 0).sum()}')
    ok = all(res)
    print(f'RESULT {"PASS" if ok else "FAIL"} ({sum(res)}/{len(res)})', flush=True)
    sys.stdout.flush()
    import os
    os._exit(0 if ok else 1)


if __name__ == '__main__':
    main()
