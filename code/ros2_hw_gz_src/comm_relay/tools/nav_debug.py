#!/usr/bin/env python3
"""Nav2(DWB) 가 왜 안 움직이나 진단 (Gazebo, 개발용). sim + SLAM + Nav2 (+ 지형) 만 떠 있을 때, comm_relay 없이.

  python3 nav_debug.py
a1_1 (스폰 (0, -1.1), 남쪽을 봄)에 목표 3 개를 차례로: A 정면 0.75 m, B 오른쪽 앞(약 60° 돌아야), C 왼쪽 뒤(약 135°).
계획(ComputePathToPose) -> 추종(FollowPath, 최대 60 s). 컨트롤러 출력 /a1_1/cmd_vel_nav_test 를 이 도구가
/a1_1/cmd_vel 로 넘긴다(0.3 s 끊기면 0). 목표마다 출력:
  결과, 걸린 시간, 실제 이동, 명령 분포(정지 / 제자리 회전 / 전진), 정지 명령일 때 odom 속도(다리 odom 흔들림이
  DWB 속도 창을 막는지), DWB 평가(/a1_1/evaluation)가 있으면: 정지를 고른 주기에서 전진 후보가 불법(충돌)인 비율,
  전진 후보가 정지보다 점수가 나쁜 이유가 된 비평가(critic).
2026-11-08 탐색: a1_1 이 스폰 근처에서 주변 위험·치명 칸 0 인데 Nav2 명령 vx 평균 0.000~0.013 으로 멈춤.
"""
import math
import sys
import threading
import time
from collections import Counter

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

try:
    from dwb_msgs.msg import LocalPlanEvaluation
except ImportError:
    LocalPlanEvaluation = None

NS, SPAWN = 'a1_1', (0.0, -1.1)
GOALS = [('A 정면 0.75 m', (0.0, -1.85)), ('B 오른쪽 앞', (-0.7, -1.5)), ('C 왼쪽 뒤', (0.75, -0.75))]


class D(Node):
    def __init__(self):
        super().__init__('nav_debug')
        self.cmd = None
        self.cmd_t = 0.0
        self.log = []                         # (t, vx, wz, odom_vx, odom_vy, odom_wz)
        self.odom = (0.0, 0.0, 0.0)
        self.gt = None
        self.evals = []
        self.ev0 = 0
        self.create_subscription(Twist, f'/{NS}/cmd_vel_nav_test', self.on_cmd, 10)
        self.create_subscription(Odometry, f'/{NS}/odom', self.on_odom, 20)
        self.create_subscription(Odometry, f'/{NS}/odom/ground_truth', self.on_gt, 10)
        if LocalPlanEvaluation is not None:
            self.create_subscription(LocalPlanEvaluation, f'/{NS}/evaluation', self.evals.append, 10)
        self.pub = self.create_publisher(Twist, f'/{NS}/cmd_vel', 10)
        self.plan = ActionClient(self, ComputePathToPose, f'/{NS}/compute_path_to_pose')
        self.follow = ActionClient(self, FollowPath, f'/{NS}/follow_path')
        self.create_timer(0.05, self.relay)

    def on_cmd(self, m):
        self.cmd, self.cmd_t = (m.linear.x, m.angular.z), time.monotonic()
        self.log.append((time.monotonic(), m.linear.x, m.angular.z) + self.odom)

    def on_odom(self, m):
        t = m.twist.twist
        self.odom = (t.linear.x, t.linear.y, t.angular.z)

    def on_gt(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.gt = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def relay(self):
        tw = Twist()
        if self.cmd is not None and time.monotonic() - self.cmd_t < 0.3:
            tw.linear.x, tw.angular.z = self.cmd
        self.pub.publish(tw)


def wait(fut, t):
    end = time.monotonic() + t
    while not fut.done() and time.monotonic() < end:
        time.sleep(0.05)
    return fut.result() if fut.done() else None


def analyse(n, t0, t1):
    L = [r for r in n.log if t0 <= r[0] <= t1]
    if not L:
        return '  명령 0 개 (컨트롤러 출력 없음)'
    stop = [r for r in L if abs(r[1]) < 0.01 and abs(r[2]) < 0.01]
    rot = [r for r in L if abs(r[1]) < 0.01 <= abs(r[2])]
    fwd = [r for r in L if abs(r[1]) >= 0.01]
    out = [f'  명령 {len(L)} 개: 정지 {100 * len(stop) / len(L):.0f}%, 제자리 회전 {100 * len(rot) / len(L):.0f}%, '
           f'전진 {100 * len(fwd) / len(L):.0f}% (전진 vx 평균 {sum(r[1] for r in fwd) / max(1, len(fwd)):.3f})']
    if stop:
        ovx = [r[3] for r in stop]
        ovy = [r[4] for r in stop]
        out.append(f'  정지 명령 때 odom 속도: vx {min(ovx):+.3f}~{max(ovx):+.3f}, vy {min(ovy):+.3f}~{max(ovy):+.3f}')
    E = n.evals[n.ev0:]
    if LocalPlanEvaluation is None:
        out.append('  DWB 평가: dwb_msgs 없음')
    elif not E:
        out.append('  DWB 평가: /a1_1/evaluation 메시지 없음 (publish_evaluation 꺼짐?)')
    else:
        zero_best = illegal_fwd = stop_best = 0
        blame, why, rblame, rwhy = Counter(), Counter(), Counter(), Counter()
        for e in E:
            if not e.twists or e.best_index >= len(e.twists):
                continue
            best = e.twists[e.best_index]
            if abs(best.traj.velocity.x) >= 0.01:
                continue
            zero_best += 1
            if abs(best.traj.velocity.theta) < 0.01:      # 그냥 섰음: 제자리 회전 후보는 왜 졌나
                stop_best += 1
                rot = [s for s in e.twists if abs(s.traj.velocity.x) < 0.01 and abs(s.traj.velocity.theta) >= 0.05]
                rlegal = [s for s in rot if s.total >= 0]
                if rot and not rlegal:
                    rwhy.update(s.scores[0].name for s in rot if s.scores)
                elif rlegal:
                    f = min(rlegal, key=lambda s: s.total)
                    bs = {c.name: c.raw_score * c.scale for c in best.scores}
                    for c in f.scores:
                        dd = c.raw_score * c.scale - bs.get(c.name, 0.0)
                        if dd > 0:
                            rblame[c.name] += dd
            fw = [s for s in e.twists if s.traj.velocity.x >= 0.05]
            legal = [s for s in fw if s.total >= 0]
            if fw and not legal:
                illegal_fwd += 1
                why.update(s.scores[0].name for s in fw if s.scores)   # 불법이면 점수 1 개 = 예외 던진 비평가
                continue
            if legal:
                f = min(legal, key=lambda s: s.total)
                bs = {c.name: c.raw_score * c.scale for c in best.scores}
                for c in f.scores:
                    d = c.raw_score * c.scale - bs.get(c.name, 0.0)
                    if d > 0:
                        blame[c.name] += d
        out.append(f'  DWB 평가 {len(E)} 주기: 정지/회전을 고른 주기 {zero_best}, 그중 전진 후보가 전부 불법 {illegal_fwd}'
                   + (' (불법으로 만든 비평가: ' + ', '.join(f'{k} {v}' for k, v in why.most_common(3)) + ')' if why else ''))
        if blame:
            out.append('  전진이 정지보다 나빴던 이유(가중 점수 차 합): '
                       + ', '.join(f'{k} {v:.1f}' for k, v in blame.most_common(4)))
        if stop_best:
            out.append(f'  그냥 선 주기 {stop_best}: 제자리 회전 후보가 전부 불법 '
                       + (', '.join(f'{k} {v}' for k, v in rwhy.most_common(3)) if rwhy else '0')
                       + ('; 회전이 정지보다 나빴던 이유: ' + ', '.join(f'{k} {v:.1f}' for k, v in rblame.most_common(4))
                          if rblame else ''))
    return '\n'.join(out)


def main():
    rclpy.init()
    n = D()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    print('== nav_debug a1_1', flush=True)
    if not (n.plan.wait_for_server(timeout_sec=20) and n.follow.wait_for_server(timeout_sec=20)):
        print('Nav2 액션 서버 없음')
        return
    t = time.monotonic()
    while n.gt is None and time.monotonic() - t < 10:
        time.sleep(0.1)
    for name, (wx, wy) in GOALS:
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.pose.position.x, g.pose.position.y = wx - SPAWN[0], wy - SPAWN[1]
        g.pose.orientation.w = 1.0
        req = ComputePathToPose.Goal()
        req.goal, req.planner_id, req.use_start = g, 'GridBased', False
        h = wait(n.plan.send_goal_async(req), 10)
        res = wait(h.get_result_async(), 10) if h is not None and h.accepted else None
        if res is None or not res.result.path.poses:
            print(f'{name}: 계획 실패 ({None if res is None else res.status})', flush=True)
            continue
        path = res.result.path
        x0, y0, _ = n.gt
        n.ev0 = len(n.evals)
        t0 = time.monotonic()
        fg = FollowPath.Goal()
        fg.path, fg.controller_id, fg.goal_checker_id = path, 'FollowPath', 'general_goal_checker'
        fh = wait(n.follow.send_goal_async(fg), 10)
        fr = wait(fh.get_result_async(), 60) if fh is not None and fh.accepted else None
        if fr is None and fh is not None:
            fh.cancel_goal_async()
        t1 = time.monotonic()
        moved = math.hypot(n.gt[0] - x0, n.gt[1] - y0)
        left = math.hypot(n.gt[0] - wx, n.gt[1] - wy)
        st = {4: '성공', 6: '실패(ABORTED)', 5: '취소'}.get(fr.status if fr else -1, '60 s 초과')
        print(f'{name} -> 월드 ({wx}, {wy}): {st}, {t1 - t0:.0f} s, 경로 {len(path.poses)}점, 실제 이동 {moved:.2f} m, '
              f'남은 거리 {left:.2f} m, 끝 방향 {math.degrees(n.gt[2]):+.0f}° (목표 +0°)', flush=True)
        print(analyse(n, t0, t1), flush=True)
        n.cmd = None
        time.sleep(2.0)
    sys.stdout.flush()
    import os
    os._exit(0)


if __name__ == '__main__':
    main()
