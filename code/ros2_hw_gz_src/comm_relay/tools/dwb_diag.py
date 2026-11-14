#!/usr/bin/env python3
"""DWB 0 명령 진단 (2026-11-10, fix7 미션: 열린 곳에서 Nav2 vx·wz 가 둘 다 0 인 채 8 s -> '0 명령 감시' 실패 9 번, 원인 모름).

실행기 FOLLOWING 중 Nav2 명령(/<ns>/cmd_vel_nav_test)이 |vx|,|wz| < 0.02 로 시뮬 HOLD(3) 초 이어지면 그 순간 DWB 평가
(/<ns>/evaluation, dwb_msgs/LocalPlanEvaluation — nav2_controllers 의 FollowPath.publish_evaluation, A1_DWB_DIAG=1 때만 켬)를 기록:
  - 궤적 수, 유효 수(total >= 0), 무효 궤적이 마지막으로 걸린 판정 항목(critic) 별 수
  - DWB 가 고른 궤적(best_index)의 속도와 critic 별 점수(raw x scale)
  - 유효한 것 중 앞으로 가는(vx >= 0.05) 최고 궤적의 속도와 critic 별 점수 -> 정지가 왜 이겼는지
출력: $REC_DIR/dwb_diag.jsonl (한 줄 = 한 번), 표준 출력에 한 줄 요약. 사용: python3 dwb_diag.py  (REC_DIR 없으면 현재 폴더)
"""
import json
import os
from collections import Counter

import rclpy
from dwb_msgs.msg import LocalPlanEvaluation
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

ROBOTS = ('a1_1', 'a1_2')
EPS, HOLD = 0.02, 3.0


def breakdown(ts):
    return {s.name: round(s.raw_score * s.scale, 3) for s in ts.scores}


class Diag(Node):
    def __init__(self):
        super().__init__('dwb_diag')
        self.out = os.path.join(os.environ.get('REC_DIR', '.'), 'dwb_diag.jsonl')
        self.sim = 0.0
        self.create_subscription(Clock, '/clock', lambda m: setattr(self, 'sim', m.clock.sec + m.clock.nanosec * 1e-9), 10)
        self.d = {ns: dict(state='', last_move=None, ev=None, logged=False) for ns in ROBOTS}
        for ns in ROBOTS:
            self.create_subscription(String, f'/{ns}/nav_executor/state', lambda m, ns=ns: self.on_state(ns, m.data), 10)
            self.create_subscription(Twist, f'/{ns}/cmd_vel_nav_test', lambda m, ns=ns: self.on_cmd(ns, m), 10)
            self.create_subscription(LocalPlanEvaluation, f'/{ns}/evaluation',
                                     lambda m, ns=ns: self.d[ns].__setitem__('ev', m), 1)
        self.create_timer(0.5, self.tick)

    def on_state(self, ns, s):
        d = self.d[ns]
        if s != d['state']:
            d['state'], d['last_move'], d['logged'] = s, self.sim, False

    def on_cmd(self, ns, m):
        if abs(m.linear.x) >= EPS or abs(m.angular.z) >= EPS:
            self.d[ns]['last_move'], self.d[ns]['logged'] = self.sim, False

    def tick(self):
        for ns, d in self.d.items():
            if d['state'] != 'FOLLOWING' or d['logged'] or d['last_move'] is None or self.sim - d['last_move'] < HOLD:
                continue
            d['logged'] = True
            ev = d['ev']
            rec = dict(robot=ns, sim=round(self.sim, 1), zero_for=round(self.sim - d['last_move'], 1))
            if ev is None or not ev.twists:
                rec['eval'] = '없음 (evaluation 토픽 안 옴)'
            else:
                tw = list(ev.twists)
                valid = [t for t in tw if t.total >= 0]
                fail_by = Counter(t.scores[-1].name for t in tw if t.total < 0 and t.scores)
                best = tw[ev.best_index] if ev.best_index < len(tw) else None
                fwd = sorted((t for t in valid if t.traj.velocity.x >= 0.05), key=lambda t: t.total)
                rec.update(n=len(tw), valid=len(valid), invalid_by=dict(fail_by),
                           eval_age=round(self.sim - (ev.header.stamp.sec + ev.header.stamp.nanosec * 1e-9), 2))
                if best is not None:
                    rec['best'] = dict(vx=round(best.traj.velocity.x, 3), wz=round(best.traj.velocity.theta, 3),
                                       total=round(best.total, 3), critics=breakdown(best))
                if fwd:
                    f = fwd[0]
                    rec['best_forward'] = dict(vx=round(f.traj.velocity.x, 3), wz=round(f.traj.velocity.theta, 3),
                                               total=round(f.total, 3), critics=breakdown(f))
            with open(self.out, 'a', encoding='utf-8') as fh:
                fh.write(json.dumps(rec, ensure_ascii=False) + '\n')
            b, f = rec.get('best', {}), rec.get('best_forward', {})
            print(f"[dwb_diag] {ns} sim {rec['sim']} 0 명령 {rec['zero_for']} s: 궤적 {rec.get('n')} 유효 {rec.get('valid')} "
                  f"무효 원인 {rec.get('invalid_by')} | 고른 것 vx {b.get('vx')} wz {b.get('wz')} 합 {b.get('total')} "
                  f"| 전진 최고 vx {f.get('vx')} 합 {f.get('total')} {f.get('critics')}", flush=True)


def main():
    rclpy.init()
    rclpy.spin(Diag())


if __name__ == '__main__':
    main()
