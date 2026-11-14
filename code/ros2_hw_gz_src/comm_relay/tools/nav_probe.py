#!/usr/bin/env python3
"""Nav2 계획 실패 진단 (Gazebo + SLAM + Nav2 가 떠 있을 때): python3 nav_probe.py [ns] [태그]

출력
  - SLAM 지도(/<ns>/map): 크기·해상도·원점, 알려진 칸 수
  - 전역 비용지도(/<ns>/global_costmap/costmap): 크기·원점
  - 로봇 위치(map 프레임 = TF map->odom + /odom)와 시험 목표들의 칸 값: 지도(-1 미탐색, 0 빈칸, 100 벽) / 비용(-1 미탐색, 99 내접, 100 치명)
  - 각 목표로 ComputePathToPose 를 실제로 불러 결과(상태, 점 수)
목표(월드 좌표, 시험 nav 와 같음): 앞 0.5 m, (1.0, 2.3), (-1.0, 2.3)  -> 로봇 map 좌표로 바꿔 보냄
"""
import math
import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

NS = sys.argv[1] if len(sys.argv) > 1 else 'a1_2'
TAG = sys.argv[2] if len(sys.argv) > 2 else ''
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class P(Node):
    def __init__(self):
        super().__init__('nav_probe')
        self.map = self.cost = self.odom = self.gt = self.m2o = None
        self.create_subscription(OccupancyGrid, f'/{NS}/map', lambda m: setattr(self, 'map', m), LATCHED)
        self.create_subscription(OccupancyGrid, f'/{NS}/global_costmap/costmap', lambda m: setattr(self, 'cost', m), LATCHED)
        self.create_subscription(Odometry, f'/{NS}/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(Odometry, f'/{NS}/odom/ground_truth', lambda m: setattr(self, 'gt', m), 10)
        self.create_subscription(TFMessage, f'/{NS}/tf', self.on_tf, 50)
        self.plan = ActionClient(self, ComputePathToPose, f'/{NS}/compute_path_to_pose')

    def on_tf(self, m):
        for t in m.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                self.m2o = t.transform

    def robot_map(self):
        p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
        ox, oy, oth = p.x, p.y, yaw(q)
        if self.m2o is None:
            return ox, oy, oth, 'map->odom 없음(항등 가정)'
        tx, ty, tth = self.m2o.translation.x, self.m2o.translation.y, yaw(self.m2o.rotation)
        return (tx + math.cos(tth) * ox - math.sin(tth) * oy, ty + math.sin(tth) * ox + math.cos(tth) * oy,
                tth + oth, f'map->odom ({tx:.2f}, {ty:.2f}, {math.degrees(tth):.1f}°)')

    def world_to_map(self, wx, wy):
        g, gq = self.gt.pose.pose.position, self.gt.pose.pose.orientation
        rx, ry, rth, _ = self.robot_map()
        d = rth - yaw(gq)
        dx, dy = wx - g.x, wy - g.y
        return rx + math.cos(d) * dx - math.sin(d) * dy, ry + math.sin(d) * dx + math.cos(d) * dy


def cell(grid, x, y):
    if grid is None:
        return 'n/a'
    i = int(math.floor((x - grid.info.origin.position.x) / grid.info.resolution))
    j = int(math.floor((y - grid.info.origin.position.y) / grid.info.resolution))
    if not (0 <= i < grid.info.width and 0 <= j < grid.info.height):
        return '범위 밖'
    v = grid.data[j * grid.info.width + i]
    r = 2      # 주변 5x5 칸 최댓값도 (팽창/벽 근처 여부)
    near = [grid.data[jj * grid.info.width + ii] for jj in range(max(0, j - r), min(grid.info.height, j + r + 1))
            for ii in range(max(0, i - r), min(grid.info.width, i + r + 1))]
    return f'{v} (주변 최대 {max(near)}, 미탐색 {sum(1 for n in near if n < 0)}/{len(near)})'


def info(name, g):
    if g is None:
        return f'{name}: 수신 안 됨'
    known = sum(1 for v in g.data if v >= 0)
    return (f'{name}: {g.info.width}x{g.info.height} 칸, {g.info.resolution:.3f} m, 원점 ({g.info.origin.position.x:.2f}, '
            f'{g.info.origin.position.y:.2f}), 알려진 칸 {known}/{len(g.data)}, frame {g.header.frame_id}')


def main():
    rclpy.init()
    n = P()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    t0 = time.monotonic()
    while (n.odom is None or n.gt is None) and time.monotonic() - t0 < 15:
        time.sleep(0.2)
    time.sleep(3)
    print(f'== nav_probe {NS} {TAG}')
    print(info('SLAM 지도', n.map))
    print(info('전역 비용지도', n.cost))
    if n.odom is None or n.gt is None:
        print('odom/ground truth 없음')
        os._exit(1)
    rx, ry, rth, how = n.robot_map()
    g = n.gt.pose.pose.position
    print(f'로봇: 월드 ({g.x:.2f}, {g.y:.2f}) / map ({rx:.2f}, {ry:.2f}, {math.degrees(rth):.0f}°) [{how}]')
    print(f'  로봇 칸: 지도 {cell(n.map, rx, ry)} | 비용 {cell(n.cost, rx, ry)}')
    goals = [('앞 0.5 m', g.x + 0.5 * math.cos(yaw(n.gt.pose.pose.orientation)),
              g.y + 0.5 * math.sin(yaw(n.gt.pose.pose.orientation))), ('(1.0, 2.3)', 1.0, 2.3), ('(-1.0, 2.3)', -1.0, 2.3)]
    ready = n.plan.wait_for_server(timeout_sec=5.0)
    for name, wx, wy in goals:
        mx, my = n.world_to_map(wx, wy)
        line = f'  목표 {name}: map ({mx:.2f}, {my:.2f}) 지도 {cell(n.map, mx, my)} | 비용 {cell(n.cost, mx, my)}'
        if ready:
            req = ComputePathToPose.Goal()
            req.goal.header.frame_id = 'map'
            req.goal.header.stamp = n.get_clock().now().to_msg()
            req.goal.pose.position.x, req.goal.pose.position.y = mx, my
            req.goal.pose.orientation.w = 1.0
            req.planner_id = 'GridBased'
            req.use_start = False
            gh_f = n.plan.send_goal_async(req)
            while not gh_f.done():
                time.sleep(0.05)
            gh = gh_f.result()
            if gh.accepted:
                r_f = gh.get_result_async()
                while not r_f.done():
                    time.sleep(0.05)
                r = r_f.result()
                line += f' -> 계획 status {r.status}, {len(r.result.path.poses)}점'
            else:
                line += ' -> 계획 거절'
        else:
            line += ' -> 플래너 서버 없음'
        print(line, flush=True)
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
