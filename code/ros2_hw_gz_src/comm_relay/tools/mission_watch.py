#!/usr/bin/env python3
"""임무 관찰 기록 (run_mission.sh 가 부름): 10 시뮬 초마다 한 줄, 단계가 바뀔 때 한 줄, 끝에 요약.

  python3 mission_watch.py <결과 폴더> <start_set> <세트 수> <시뮬 초 한도> [통신 끔 시각] [끈 시간]
위치는 Gazebo 정답(/<이름>/odom/ground_truth) — 평가용으로만 쓴다 (로봇 노드는 안 씀).
요약: 세트별 걸린 시간·찾은 사람(정답 1.0 m 안), 넘어짐, A1-Husky / A1-A1 최소 거리, 추종 중 Husky 와의 최대 거리,
  nav_executor 복구 장치별 작동 횟수 (<결과 폴더>/mission.log 에서 셈 — 쓰이지 않는 장치를 가려내려고, 2026-11-18),
  세트별 홈 복귀 결과 (/<ns>/nav_executor/events 의 '홈 도착' / '홈 실패'),
  멈춤 분석 (motion.csv 2 Hz 기록 -> stuck_report.py: FOLLOWING 중 멈춘 시간을 보행 / Nav 로 나눔, 2026-11-19).
PASS = 끝까지 감 + 넘어짐 0 + 탐색 시간 한도(REGROUP) 0 + 맡은 세트의 사람을 전부 찾음 + 홈 실패 0.
  홈 실패: nav_executor 는 복귀를 home_retries 번 실패해도 그 자리에서 set_done 을 보내 husky_mission 은 '복귀한 A1' 로 센다.
  그래서 실제 도착은 여기서 따로 본다 (2026-11-19: 홈 실패가 있던 실행 3 개가 전에는 PASS 로 찍혔음).
"""
import csv
import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from mission_interfaces.msg import MissionState, SurvivorList
from mission_interfaces.srv import SetComm
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from stuck_report import MOTION_COLS, report

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
NAMES = ('husky', 'a1_1', 'a1_2')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
# nav_executor 복구 장치 -> 작동할 때 mission.log 에 남기는 문장 조각. * = throttle 로그라 실제보다 적게 셀 수 있음
RECOVERY = (('0 명령 감시', '0 명령 감시: FOLLOWING 중'), ('기울기 가드', '이상 -> 추종 중단'),
            ('벗어나기', '-> ESCAPING (중심'), ('옮겨 보기', '옮겨 보기 odom'),
            ('폐경로+물러남', '폐경로로 닫고'), ('폐경로 거부', '폐경로(갇혔던 자리) 지남'),
            ('거꾸로 추종', '거꾸로 추종'), ('상대 구역 경로', '경로가 상대 구역으로 감'),
            ('목표 보정', '목표가 장애물 팽창 영역 ->'), ('목표 보정 버림', '여유 칸 없음'),
            ('반복 실패 차단*', '번 실패 -> 블랙리스트'), ('같은 자리 3 번*', '도착했지만 프런티어가 남음'),
            ('도착 목표 다시 옴*', '다시 옴 (프런티어가 안 지워짐)'))


def recovery_counts(log, robots=NAMES[1:]):
    """mission.log 에서 로봇별 복구 장치 작동 횟수 {ns: {이름: 줄 수}}. 로그를 못 읽으면 None."""
    try:
        with open(log, errors='replace') as f:
            lines = [line for line in f if '[nav_executor]' in line]
    except OSError:
        return None
    return {ns: {name: sum(1 for line in lines if f'[{ns}]' in line and pat in line) for name, pat in RECOVERY}
            for ns in robots}


class Watch(Node):
    def __init__(self, out, k0, n_sets, dur, off_at, off_for):
        super().__init__('mission_watch', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        with open(os.path.join(get_package_share_directory('disaster_sim'), 'config', 'mission.yaml')) as f:
            self.cfg = yaml.safe_load(f)
        self.out, self.k0, self.k_end, self.dur, self.off = out, k0, k0 + n_sets, dur, (off_at, off_for)
        self.pos, self.z, self.st, self.mission, self.people, self.slam = {}, {}, {}, None, [], {}
        self.max_slam = {ns: 0.0 for ns in NAMES[1:]}
        self.t0 = self.t_explore = None
        self.marks, self.falls, self.finished = [], set(), False
        self.min_d = {'a1_1-husky': 9.0, 'a1_2-husky': 9.0, 'a1_1-a1_2': 9.0}
        self.max_follow = 0.0
        self.home = {}               # (세트, 로봇) -> '도착' | '실패' (마지막 것)
        self.yaw, self.cmd, self.src, self.od = {}, {}, {}, {}     # motion.csv 용: 정답 yaw, 중재기 출력 (vx, wz), 출처, EKF odom (x, y, yaw)
        self.sl_yaw = {}                                            # motion.csv 용: SLAM 공유 자세 yaw
        for n in NAMES:
            self.create_subscription(Odometry, f'/{n}/odom/ground_truth', lambda m, n=n: self.on_odom(n, m), 10)
        for ns in NAMES[1:]:
            self.create_subscription(String, f'/{ns}/nav_executor/state', lambda m, n=ns: self.st.__setitem__(n, m.data), 10)
            self.create_subscription(PoseStamped, f'/{ns}/pose_shared', lambda m, n=ns: (self.slam.__setitem__(
                n, (m.pose.position.x, m.pose.position.y)), self.sl_yaw.__setitem__(n, yaw_of(m.pose.orientation))), 10)
            self.create_subscription(String, f'/{ns}/comm_mode', lambda m, n=ns: self.st.__setitem__(n + '_comm', m.data), LATCHED)
            self.create_subscription(String, f'/{ns}/nav_executor/events', lambda m, n=ns: self.on_event(n, m.data), 10)
            self.create_subscription(Twist, f'/{ns}/cmd_vel', lambda m, n=ns: self.cmd.__setitem__(n, (m.linear.x, m.angular.z, m.linear.y)), 10)
            self.create_subscription(String, f'/{ns}/cmd_vel_source', lambda m, n=ns: self.src.__setitem__(n, m.data), LATCHED)
            self.create_subscription(Odometry, f'/{ns}/odom', lambda m, n=ns: self.od.__setitem__(
                n, (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))), 10)
        self.create_subscription(String, '/husky/driver_state', lambda m: self.st.__setitem__('husky', m.data), LATCHED)
        self.create_subscription(MissionState, '/mission/state', self.on_mission, LATCHED)
        self.create_subscription(SurvivorList, '/husky/survivors', lambda m: setattr(
            self, 'people', [[round(p.x, 2), round(p.y, 2)] for p in m.people]), LATCHED)
        self.comm_cli, self.comm_call = self.create_client(SetComm, '/husky/set_comm'), None     # 통신 시험 (COMM_OFF_AT)
        self.csv = csv.writer(open(os.path.join(out, 'timeline.csv'), 'w', newline=''))
        self.csv.writerow(['t', 'phase', 'set', 'comm', 'husky_x', 'husky_y', 'a1_1_x', 'a1_1_y', 'a1_2_x', 'a1_2_y',
                           'husky', 'a1_1', 'a1_2', 'a1_1_comm', 'a1_2_comm', 'people', 'a1_1_slam_err', 'a1_2_slam_err'])
        self.create_timer(1.0, self.tick)
        self.last_row = -99.0
        # 2 Hz 움직임 기록 -> stuck_report.py 가 '멈춤'을 보행 / Nav 로 나눈다 (명령은 나가는데 몸이 안 가는지, 명령이 머뭇거리는지)
        self.motion_f = open(os.path.join(out, 'motion.csv'), 'w', newline='')
        self.motion = csv.writer(self.motion_f)
        self.motion.writerow(['t', 'phase', 'set'] + [f'{ns}_{k}' for ns in NAMES[1:] for k in MOTION_COLS])
        self.create_timer(0.5, self.motion_tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def motion_tick(self):
        d = self.mission
        if d is None or self.t0 is None or any(ns not in self.pos for ns in NAMES[1:]):
            return
        row = [f'{self.t():.1f}', d['phase'], d['set']]
        for ns in NAMES[1:]:
            vx, wz, vy = self.cmd.get(ns, (0.0, 0.0, 0.0))
            ox, oy, oyaw = self.od.get(ns, (float('nan'),) * 3)
            sx, sy = self.slam.get(ns, (float('nan'),) * 2)
            row += [self.st.get(ns, '-'), self.src.get(ns, '-'), f'{vx:.3f}', f'{wz:.3f}',
                    f'{self.pos[ns][0]:.3f}', f'{self.pos[ns][1]:.3f}', f'{self.yaw.get(ns, 0.0):.3f}',
                    f'{ox:.3f}', f'{oy:.3f}', f'{oyaw:.3f}', f'{vy:.3f}',
                    f'{sx:.3f}', f'{sy:.3f}', f"{self.sl_yaw.get(ns, float('nan')):.4f}"]
        self.motion.writerow(row)

    def on_odom(self, n, m):
        p = m.pose.pose.position
        self.pos[n], self.z[n] = (p.x, p.y), p.z
        q = m.pose.pose.orientation
        self.yaw[n] = yaw_of(q)
        if n != 'husky' and 1 - 2 * (q.x * q.x + q.y * q.y) < 0.7:     # 기울기 45° 넘음
            if n not in self.falls:
                print(f'!! {n} 넘어짐 t={self.t():.0f} s ({p.x:.2f}, {p.y:.2f})', flush=True)
            self.falls.add(n)

    def t(self):
        return self.now() - self.t0 if self.t0 is not None else 0.0

    def on_event(self, ns, text):
        if text in ('홈 도착', '홈 실패') and self.mission is not None:
            self.home[(self.mission['set'], ns)] = text[2:]
            if text == '홈 실패':
                print(f"!! {ns} 홈 실패 t={self.t():.0f} s (세트 {self.mission['set']})", flush=True)

    def on_mission(self, m):
        d = dict(phase=m.phase, set=m.set_index, name=m.set_name, comm=int(m.comm))
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
            busy = self.comm_call is not None and not self.comm_call[0].done() and self.now() - self.comm_call[1] < 5.0   # 답 없는 요청은 5 s 뒤 버리고 다시
            if want != str(d['comm']) and not busy and self.comm_cli.service_is_ready():
                self.comm_call = (self.comm_cli.call_async(SetComm.Request(enabled=want == '1')), self.now())
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
        robots = NAMES[1:]
        print(f'  홈 복귀 ({" / ".join(robots)}): ' + ', '.join(
            f'세트 {k} ' + '/'.join(self.home.get((k, ns), '-') for ns in robots) for k in range(self.k0, self.k_end)
            if k < len(self.cfg['sets'])))
        home_fail = sum(1 for v in self.home.values() if v == '실패')
        print('  최소 중심 거리: ' + ', '.join(f'{k} {v:.2f} m' for k, v in self.min_d.items()) + f', 추종 중 Husky 와 최대 {self.max_follow:.2f} m')
        print('  SLAM 공유 자세 최대 오차: ' + ', '.join(f'{k} {v:.2f} m' for k, v in self.max_slam.items()))
        self.motion_f.flush()
        stuck = report(os.path.join(self.out, 'motion.csv'))
        with open(os.path.join(self.out, 'stuck.txt'), 'w') as f:
            f.write(stuck + '\n')
        print('  ' + stuck.splitlines()[0])        # 전체 한 줄 (세트·로봇별 표는 stuck.txt)
        rc = recovery_counts(os.path.join(self.out, 'mission.log'))
        if rc:
            print('  복구 장치 작동 (mission.log 줄 수, * 는 throttle 로그라 덜 셀 수 있음):')
            for ns, c in rc.items():
                print(f'    {ns}: ' + ', '.join(f'{k} {v}' for k, v in c.items()))
        last = self.mission or {}
        done = last.get('phase') == 'DONE' or last.get('set', 0) >= self.k_end
        timeouts = sum(1 for _, ph, _ in self.marks if ph == 'REGROUP')                 # 탐색 시간 한도로 불러들인 세트 수
        ok = done and not self.falls and timeouts == 0 and hit == len(truth) and home_fail == 0
        print(f"RESULT {'PASS' if ok else 'FAIL'} (마지막 단계 {last.get('phase')} 세트 {last.get('set')}, "
              f"한도 초과 세트 {timeouts}, 사람 {hit}/{len(truth)}, 홈 실패 {home_fail})")


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
