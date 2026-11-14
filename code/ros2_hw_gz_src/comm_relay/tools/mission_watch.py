#!/usr/bin/env python3
"""임무 관찰 기록 (run_mission.sh 가 부름): 10 시뮬 초마다 한 줄, 단계가 바뀔 때 한 줄, 끝에 요약.

  python3 mission_watch.py <결과 폴더> <start_set> <세트 수> <시뮬 초 한도> [통신 끔 시각] [끈 시간]
위치는 Gazebo 정답(/<이름>/odom/ground_truth) — 평가용으로만 쓴다 (로봇 노드는 안 씀).
요약: 세트별 걸린 시간·찾은 사람(정답 1.0 m 안), 넘어짐, A1-Husky / A1-A1 최소 거리, 추종 중 Husky 와의 최대 거리.
PASS = 끝까지 감 + 넘어짐 0 + 탐색 시간 한도(REGROUP) 0 + 맡은 세트의 사람을 전부 찾음.
"""
import csv
import json
import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
NAMES = ('husky', 'a1_1', 'a1_2')


class Watch(Node):
    def __init__(self, out, k0, n_sets, dur, off_at, off_for):
        super().__init__('mission_watch', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        with open(os.path.join(get_package_share_directory('disaster_sim'), 'config', 'mission.yaml')) as f:
            self.cfg = yaml.safe_load(f)
        self.k0, self.k_end, self.dur, self.off = k0, k0 + n_sets, dur, (off_at, off_for)
        self.pos, self.z, self.st, self.mission, self.people, self.slam = {}, {}, {}, None, [], {}
        self.max_slam = {ns: 0.0 for ns in NAMES[1:]}
        self.t0 = self.t_explore = None
        self.marks, self.falls, self.finished = [], set(), False
        self.min_d = {'a1_1-husky': 9.0, 'a1_2-husky': 9.0, 'a1_1-a1_2': 9.0}
        self.max_follow = 0.0
        for n in NAMES:
            self.create_subscription(Odometry, f'/{n}/odom/ground_truth', lambda m, n=n: self.on_odom(n, m), 10)
        for ns in NAMES[1:]:
            self.create_subscription(String, f'/{ns}/nav_executor/state', lambda m, n=ns: self.st.__setitem__(n, m.data), 10)
            self.create_subscription(PoseStamped, f'/{ns}/pose_shared', lambda m, n=ns: self.slam.__setitem__(
                n, (m.pose.position.x, m.pose.position.y)), 10)
            self.create_subscription(String, f'/{ns}/comm_mode', lambda m, n=ns: self.st.__setitem__(n + '_comm', m.data), LATCHED)
        self.create_subscription(String, '/husky/driver_state', lambda m: self.st.__setitem__('husky', m.data), LATCHED)
        self.create_subscription(String, '/mission/state', self.on_mission, LATCHED)
        self.create_subscription(String, '/husky/survivors', lambda m: setattr(self, 'people', json.loads(m.data)), LATCHED)
        self.pub_cmd = self.create_publisher(String, '/husky/cmd', 10)
        self.csv = csv.writer(open(os.path.join(out, 'timeline.csv'), 'w', newline=''))
        self.csv.writerow(['t', 'phase', 'set', 'comm', 'husky_x', 'husky_y', 'a1_1_x', 'a1_1_y', 'a1_2_x', 'a1_2_y',
                           'husky', 'a1_1', 'a1_2', 'a1_1_comm', 'a1_2_comm', 'people', 'a1_1_slam_err', 'a1_2_slam_err'])
        self.create_timer(1.0, self.tick)
        self.last_row = -99.0

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, n, m):
        p = m.pose.pose.position
        self.pos[n], self.z[n] = (p.x, p.y), p.z
        q = m.pose.pose.orientation
        if n != 'husky' and 1 - 2 * (q.x * q.x + q.y * q.y) < 0.7:     # 기울기 45° 넘음
            if n not in self.falls:
                print(f'!! {n} 넘어짐 t={self.t():.0f} s ({p.x:.2f}, {p.y:.2f})', flush=True)
            self.falls.add(n)

    def t(self):
        return self.now() - self.t0 if self.t0 is not None else 0.0

    def on_mission(self, m):
        d = json.loads(m.data)
        prev = self.mission
        self.mission = d
        if prev is None or (prev['phase'], prev['set']) != (d['phase'], d['set']):
            if self.t0 is None and d['phase'] != 'IDLE':
                self.t0 = self.now()
            if d['phase'] == 'EXPLORE' and self.t_explore is None:
                self.t_explore = self.now()
            self.marks.append((self.t(), d['phase'], d['set']))
            print(f"--- t={self.t():6.0f} s  {d['phase']} {d['name']} (세트 {d['set']}), 사람 {len(self.people)}", flush=True)
            if d['phase'] == 'DONE' or (d['phase'] == 'TRANSIT' and d['set'] >= self.k_end):
                self.finished = True

    def tick(self):
        if self.mission is None or len(self.pos) < 3:
            return
        d, P = self.mission, self.pos
        for a, b in (('a1_1', 'husky'), ('a1_2', 'husky'), ('a1_1', 'a1_2')):
            self.min_d[f'{a}-{b}'] = min(self.min_d[f'{a}-{b}'], math.dist(P[a], P[b]))
        if d['phase'] == 'TRANSIT':
            self.max_follow = max(self.max_follow, math.dist(P['a1_1'], P['husky']), math.dist(P['a1_2'], P['husky']))
        off_at, off_for = self.off
        if off_at > 0 and self.t_explore is not None:
            e = self.now() - self.t_explore
            want = '0' if off_at <= e < off_at + off_for else '1'
            if want != str(d['comm']):
                self.pub_cmd.publish(String(data=want))
        err = {ns: math.dist(P[ns], self.slam[ns]) if ns in self.slam else float('nan') for ns in NAMES[1:]}   # SLAM 공유 자세 - 정답
        for ns, e in err.items():
            self.max_slam[ns] = max(self.max_slam[ns], e if e == e else 0.0)
        t = self.t()
        if t - self.last_row >= 10.0:
            self.last_row = t
            row = [f'{t:.0f}', d['phase'], d['set'], d['comm']] + [f'{v:.2f}' for n in NAMES for v in P[n]] + \
                  [self.st.get(k, '-') for k in ('husky', 'a1_1', 'a1_2', 'a1_1_comm', 'a1_2_comm')] + [len(self.people)] + \
                  [f'{err[ns]:.2f}' for ns in NAMES[1:]]
            self.csv.writerow(row)
            print(' '.join(str(c) for c in row), flush=True)
        if t > self.dur:
            print(f'시뮬 {self.dur:.0f} s 한도 -> 끝')
            self.finished = True

    def summary(self):
        print('\n=== 요약')
        ends = self.marks[1:] + [(self.t(), 'END', -1)]
        for (t, ph, k), (t2, _, _) in zip(self.marks, ends):
            print(f'  {ph:8s} 세트 {k}: {t:6.0f} -> {t2:6.0f} s ({t2 - t:5.0f} s)')
        sets = self.cfg['sets'][self.k0:self.k_end]              # 이번 실행이 맡은 세트의 사람만 센다
        truth = [(x, y) for x, y in self.cfg['people_truth']
                 if any(z[0] <= x <= z[1] and z[2] <= y <= z[3] for s in sets for z in s['zones'].values())]
        hit = sum(1 for tx, ty in truth if any(math.hypot(tx - x, ty - y) <= 1.0 for x, y in self.people))
        print(f'  사람 보고 {len(self.people)} 건, 정답 {len(truth)} 명 중 1.0 m 안 일치 {hit} 명: {self.people}')
        print(f'  넘어짐: {sorted(self.falls) or "없음"}')
        print('  최소 중심 거리: ' + ', '.join(f'{k} {v:.2f} m' for k, v in self.min_d.items()) + f', 추종 중 Husky 와 최대 {self.max_follow:.2f} m')
        print('  SLAM 공유 자세 최대 오차: ' + ', '.join(f'{k} {v:.2f} m' for k, v in self.max_slam.items()))
        last = self.mission or {}
        done = last.get('phase') == 'DONE' or last.get('set', 0) >= self.k_end
        timeouts = sum(1 for _, ph, _ in self.marks if ph == 'REGROUP')                 # 탐색 시간 한도로 불러들인 세트 수
        ok = done and not self.falls and timeouts == 0 and hit == len(truth)
        print(f"RESULT {'PASS' if ok else 'FAIL'} (마지막 단계 {last.get('phase')} 세트 {last.get('set')}, "
              f"한도 초과 세트 {timeouts}, 사람 {hit}/{len(truth)})")


def main():
    out, k0, n_sets, dur = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4])
    off_at, off_for = (float(sys.argv[5]), float(sys.argv[6])) if len(sys.argv) > 6 else (0.0, 0.0)
    rclpy.init()
    w = Watch(out, k0, n_sets, dur, off_at, off_for)
    try:
        while rclpy.ok() and not w.finished:
            rclpy.spin_once(w, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    w.summary()


if __name__ == '__main__':
    main()
