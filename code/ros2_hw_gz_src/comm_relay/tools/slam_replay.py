#!/usr/bin/env python3
"""SLAM 오프라인 재생 (2026-11-19): run_mission.sh BAG=1 로 남긴 bag 을 Gazebo 없이 다시 틀어 slam_toolbox 파라미터를 비교한다.

  python3 slam_replay.py <bag 폴더> <ns> [키=값 ...] [--rate R]
    예) python3 slam_replay.py results/<실행>/bag a1_1 correlation_search_space_dimension=1.0
  출력: 전체(시작 자세로 한 번 맞춤)와 세트별(탐색 시작마다 다시 맞춤) SLAM 자세 오차 평균·최대 [m].

방법: bag 의 /<ns>/tf 는 원래 slam_toolbox 의 map->odom 이 섞여 있으므로 /<ns>/tf_raw 로 받아 map->odom 만 빼고 /<ns>/tf 로 다시 낸다.
  새로 띄운 slam_toolbox(dual_slam.launch.py 와 같은 파라미터 + 키=값 덮어쓰기)가 낸 map->odom 과 bag 의 odom->base_footprint 를 합쳐
  map 자세를 만들고, 정답(/<ns>/odom/ground_truth)과 평행이동만으로 맞춰 오차를 잰다 (map 방향 = IMU ENU = 월드, shared_map_merger 와 같음).
  bag 은 시뮬 시간으로 기록돼 있어 --clock 재생이면 원래 시간축 그대로다.
"""
import json
import math
import os
import random
import subprocess
import sys
import tempfile
import time

import yaml

CFG = os.path.join(os.path.dirname(__file__), '..', '..', 'a1_exploration', 'config', 'slam_params.yaml')


def compose(a, b):
    """2D 자세 합성 a ∘ b, 자세 = (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Errors:
    """정답과 SLAM 자세를 평행이동으로 맞춘 오차. reset() 하면 다음 표본에서 다시 맞춤 (세트별)."""

    def __init__(self):
        self.anchor, self.vals = None, []

    def reset(self):
        self.anchor = None

    def add(self, gt, est):
        if self.anchor is None:
            self.anchor = (gt[0] - est[0], gt[1] - est[1])
        e = math.hypot(gt[0] - est[0] - self.anchor[0], gt[1] - est[1] - self.anchor[1])
        self.vals.append(e)
        return e

    def text(self):
        v = self.vals
        return f'평균 {sum(v) / len(v):.2f} / 최대 {max(v):.2f} m ({len(v)} 표본)' if v else '표본 없음'


def slam_params(ns, overrides):
    with open(CFG) as f:
        p = dict(yaml.safe_load(f)['/a1_1/slam_toolbox']['ros__parameters'])
    p.update(use_sim_time=True, scan_topic=f'/{ns}/scan_ready', transform_timeout=0.0,
             min_laser_range=0.2, max_laser_range=10.0)      # dual_slam.launch.py 와 같게
    p.update(overrides)
    return {f'/{ns}/slam_toolbox': {'ros__parameters': p}}


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--rate')]
    rate = next((a.split('=', 1)[1] for a in sys.argv[1:] if a.startswith('--rate=')), '1.0')
    bag, ns = args[0], args[1]
    overrides = {k: yaml.safe_load(v) for k, v in (a.split('=', 1) for a in args[2:])}
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(100, 200))      # 실행 중인 시뮬과 안 섞이게

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    tmp = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    yaml.safe_dump(slam_params(ns, overrides), tmp)
    tmp.close()
    slam = subprocess.Popen(['ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node', '--ros-args',
                             '-r', f'__ns:=/{ns}', '-r', '__node:=slam_toolbox', '--params-file', tmp.name,
                             '-r', f'/tf:=/{ns}/tf', '-r', f'/tf_static:=/{ns}/tf_static', '-r', f'/scan:=/{ns}/scan_ready',
                             '-r', f'/map:=/{ns}/map', '-r', f'/map_metadata:=/{ns}/map_metadata'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    rclpy.init()
    node = Node('slam_replay', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    pub = node.create_publisher(TFMessage, f'/{ns}/tf', 100)
    st = dict(map_odom=None, odom_base=None, set=None, phase=None)
    total, per_set = Errors(), {}

    def on_raw(m):
        keep = [t for t in m.transforms if t.header.frame_id != 'map']
        for t in keep:
            if t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint':
                r = t.transform
                st['odom_base'] = (r.translation.x, r.translation.y, yaw_of(r.rotation))
        if keep:
            pub.publish(TFMessage(transforms=keep))

    def on_tf(m):        # 새 slam_toolbox 가 낸 map->odom
        for t in m.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                r = t.transform
                st['map_odom'] = (r.translation.x, r.translation.y, yaw_of(r.rotation))

    last = [0.0]

    def on_gt(m):
        now = node.get_clock().now().nanoseconds * 1e-9
        if st['map_odom'] is None or st['odom_base'] is None or now - last[0] < 0.5 or st['phase'] not in ('EXPLORE', 'REGROUP', 'TRANSIT'):
            return
        last[0] = now
        est = compose(st['map_odom'], st['odom_base'])
        gt = (m.pose.pose.position.x, m.pose.pose.position.y)
        total.add(gt, est)
        if st['phase'] in ('EXPLORE', 'REGROUP'):
            per_set.setdefault(st['set'], Errors()).add(gt, est)

    def on_state(m):
        d = json.loads(m.data)
        st['phase'], st['set'] = d['phase'], d['set']

    node.create_subscription(TFMessage, f'/{ns}/tf_raw', on_raw, 100)
    node.create_subscription(TFMessage, f'/{ns}/tf', on_tf, 100)
    node.create_subscription(Odometry, f'/{ns}/odom/ground_truth', on_gt, 10)
    node.create_subscription(String, '/mission/state', on_state, QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    time.sleep(3.0)                                   # slam_toolbox 뜰 때까지
    topics = ['/mission/state'] + [f'/{ns}/{t}' for t in ('scan_ready', 'odom', 'tf', 'tf_static', 'imu/data', 'odom/ground_truth')]
    play = subprocess.Popen(['ros2', 'bag', 'play', bag, '--clock', '100', '-r', rate, '--topics', *topics,
                             '--remap', f'/{ns}/tf:=/{ns}/tf_raw'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        while play.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        play.terminate()
        slam.terminate()
        os.unlink(tmp.name)
        node.destroy_node()
        rclpy.try_shutdown()
    print(f'SLAM 재생 {ns} {overrides or "(기본)"}: 전체 {total.text()}')
    for k in sorted(per_set):
        print(f'  세트 {k} (탐색 시작마다 다시 맞춤): {per_set[k].text()}')


if __name__ == '__main__':
    main()
