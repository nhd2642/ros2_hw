#!/usr/bin/env python3
"""걷는 동안 CHAMP가 내린 관절 명령 vs 실제 관절 각도를 비교한다 (원인 분리용 진단).

판정 요령
  - 추종 진폭비(실제 진폭 / 명령 진폭)가 0.9 이상이고 RMS 오차가 작다  → 관절 추종은 문제 없음
        → 발 미끄러짐(마찰)이나 걸음걸이 값(gait.yaml) 쪽을 의심
  - 추종 진폭비가 낮거나 RMS 오차가 크다 → 제어 게인/제어 주기(ros_control.yaml) 쪽을 의심

사용 (gait_test.launch.py 로 월드를 띄우고, 로봇이 선 뒤):
  ros2 run a1_champ joint_tracking.py --vx 0.2 --seconds 10
"""
import argparse
import math
import sys


def analyze(cmd_samples, act_samples):
    """cmd_samples/act_samples: {joint: [값, ...]} 같은 길이 (측정 시각마다 마지막 명령과 실제 각도)"""
    rows = {}
    for j in cmd_samples:
        c, a = cmd_samples[j], act_samples[j]
        n = len(c)
        if n < 2:
            continue
        err = [ci - ai for ci, ai in zip(c, a)]
        rms = math.sqrt(sum(e * e for e in err) / n)
        c_pp, a_pp = max(c) - min(c), max(a) - min(a)
        rows[j] = dict(n=n, rms=rms, max_err=max(abs(e) for e in err),
                       cmd_pp=c_pp, act_pp=a_pp, amp_ratio=(a_pp / c_pp) if c_pp > 1e-6 else float('nan'))
    return rows


def group_of(name):
    for g in ('hip', 'thigh', 'calf'):
        if f'_{g}_' in name:
            return g
    return 'other'


def print_report(rows):
    print(f"{'관절':<16}{'RMS오차(rad)':>13}{'최대오차':>10}{'명령 진폭':>10}{'실제 진폭':>10}{'진폭비':>8}")
    for j in sorted(rows):
        r = rows[j]
        print(f"{j:<16}{r['rms']:>13.3f}{r['max_err']:>10.3f}{r['cmd_pp']:>10.3f}{r['act_pp']:>10.3f}{r['amp_ratio']:>8.2f}")
    print('\n그룹 평균')
    for g in ('hip', 'thigh', 'calf'):
        rs = [r for j, r in rows.items() if group_of(j) == g]
        if rs:
            m = lambda k: sum(x[k] for x in rs) / len(rs)
            print(f"  {g:<6} RMS {m('rms'):.3f} rad   진폭비 {m('amp_ratio'):.2f}")
    allr = list(rows.values())
    if allr:
        valid = [r['amp_ratio'] for r in allr if not math.isnan(r['amp_ratio'])]
        ratio = sum(valid) / len(valid) if valid else float('nan')
        rms = sum(r['rms'] for r in allr) / len(allr)
        print(f'\n전체 평균: RMS {rms:.3f} rad, 진폭비 {ratio:.2f}  (명령 진폭이 0인 관절은 진폭비에서 제외)')
        calf = [r for j, r in rows.items() if group_of(j) == 'calf' and not math.isnan(r['amp_ratio'])]
        if calf:
            cr = sum(r['amp_ratio'] for r in calf) / len(calf)
            if cr < 0.8:
                print(f'→ 종아리(calf) 추종이 부족 (진폭비 {cr:.2f}). 제어 주기·게인(ros_control.yaml)을 조정해 보세요.')
        if ratio >= 0.9 and rms < 0.08:
            print('→ 관절 추종은 양호. 마찰(미끄러짐)·걸음걸이 값을 먼저 조정하세요.')


def main(argv=None):
    ap = argparse.ArgumentParser(description='관절 추종 진단')
    ap.add_argument('--ns', default='a1_1')
    ap.add_argument('--vx', type=float, default=0.2)
    ap.add_argument('--wz', type=float, default=0.0)
    ap.add_argument('--seconds', type=float, default=10.0, help='측정 시간 (시뮬레이션 시간)')
    ap.add_argument('--warmup', type=float, default=3.0, help='걷기 시작 후 버리는 시간')
    args = ap.parse_args(argv)

    import time
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory

    rclpy.init()
    node = Node('joint_tracking', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    pub = node.create_publisher(Twist, f'/{args.ns}/cmd_vel', 10)
    last_cmd = {}
    cmd_s, act_s = {}, {}
    state = {'recording': False, 'n_traj': 0, 'n_js': 0}

    def on_traj(m):
        state['n_traj'] += 1
        if m.points:
            for n, v in zip(m.joint_names, m.points[0].positions):
                last_cmd[n] = v

    def on_js(m):
        state['n_js'] += 1
        if not state['recording']:
            return
        for n, v in zip(m.name, m.position):
            if n in last_cmd:
                cmd_s.setdefault(n, []).append(last_cmd[n])
                act_s.setdefault(n, []).append(v)

    node.create_subscription(JointTrajectory, f'/{args.ns}/joint_group_effort_controller/joint_trajectory', on_traj, 50)
    node.create_subscription(JointState, f'/{args.ns}/joint_states', on_js, 50)

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def run_for(sec, vx, wz, record):
        state['recording'] = record
        end, wall_end = now() + sec, time.time() + sec * 20 + 10
        while now() < end:
            m = Twist()
            m.linear.x, m.angular.z = vx, wz
            pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.time() > wall_end:
                raise RuntimeError('시뮬레이션 시간이 흐르지 않습니다.')

    try:
        t_wait = time.time()
        while state['n_js'] == 0 and time.time() - t_wait < 20.0:
            rclpy.spin_once(node, timeout_sec=0.2)
        run_for(1.0, 0.0, 0.0, False)
        if state['n_js'] == 0:
            print(f'/{args.ns}/joint_states 가 오지 않습니다. 로봇이 스폰되었는지 확인하세요.')
            return 1
        run_for(args.warmup, args.vx, args.wz, False)
        run_for(args.seconds, args.vx, args.wz, True)
        run_for(1.0, 0.0, 0.0, False)
    finally:
        pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

    if state['n_traj'] == 0:
        print('CHAMP의 관절 명령 토픽이 오지 않습니다 '
              f'(/{args.ns}/joint_group_effort_controller/joint_trajectory). `ros2 topic list` 로 이름을 확인하세요.')
        return 1
    print(f'측정: joint_states {state["n_js"]}개 수신, 명령 {state["n_traj"]}개 수신\n')
    print_report(analyze(cmd_s, act_s))
    return 0


if __name__ == '__main__':
    sys.exit(main())
