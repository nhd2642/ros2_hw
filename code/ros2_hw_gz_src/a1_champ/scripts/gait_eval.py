#!/usr/bin/env python3
"""A1 보행 평가 스크립트 (정답 위치 odom/ground_truth 와 cmd_vel 만 사용)

사용:
  # 터미널 1: ros2 launch a1_champ gait_test.launch.py spawn:=flat
  # 터미널 2:
  ros2 run a1_champ gait_eval.py --tag baseline --tests straight,turn_left,turn_right,arc_left,arc_right
  # 경사로는 spawn:=ramp 로 다시 띄운 뒤:
  ros2 run a1_champ gait_eval.py --tag baseline --tests ramp
  # 저장된 결과 비교:
  ros2 run a1_champ gait_eval.py --list

결과는 ~/gait_results/<시각>_<tag>.json 으로 저장된다. 통과 기준은 팀 합의 전 "제안값"이다 (CRITERIA).
"""
import argparse
import datetime
import glob
import json
import math
import os
import sys

# ------------------------------------------------------------------ 시험 정의
CONTROL_DT = 0.05          # 제어 주기 20 Hz (시뮬레이션 시간 기준)

TESTS = {
    # kind: distance=출발점에서 직선거리, yaw=누적 회전각, time=고정 시간, x_reach=월드 x 좌표 도달
    'straight':   dict(vx=0.2, wz=0.0,  kind='distance', target=3.0,         timeout=40.0),
    'turn_left':  dict(vx=0.0, wz=0.5,  kind='yaw',      target=math.pi / 2, timeout=30.0),
    'turn_right': dict(vx=0.0, wz=-0.5, kind='yaw',      target=-math.pi / 2, timeout=30.0),
    'arc_left':   dict(vx=0.2, wz=0.3,  kind='time',     duration=8.0),
    'arc_right':  dict(vx=0.2, wz=-0.3, kind='time',     duration=8.0),
    'ramp':       dict(vx=0.2, wz=0.0,  kind='x_reach',  target_x=4.5,       timeout=90.0),
    # 튜닝 비교용: 시간이 고정이라 설정끼리 같은 조건에서 비교된다 (권장)
    'walk':       dict(vx=0.2, wz=0.0,  kind='time',     duration=10.0),
    'spin_left':  dict(vx=0.0, wz=0.5,  kind='time',     duration=10.0),
    'spin_right': dict(vx=0.0, wz=-0.5, kind='time',     duration=10.0),
    'back':       dict(vx=-0.2, wz=0.0, kind='time',     duration=10.0),
}

# 통과 기준 (제안값, 합의 후 여기만 고치면 된다)
CRITERIA = dict(
    max_tilt_deg=25.0,        # 평지 시험에서 허용하는 최대 기울기(roll/pitch)
    ramp_max_tilt_deg=35.0,   # 경사로 시험 (10도 경사 때문에 여유)
    straight_min_speed=0.10,  # m/s
    straight_max_drift=0.30,  # m   (진행 방향에 수직인 쏠림)
    straight_max_heading_deg=15.0,
    turn_max_disp=0.30,       # m   (제자리 회전 중 위치 이동)
    arc_ratio_range=(0.7, 1.3),  # 실제 회전각 / (wz*시간)
    min_speed_ratio=0.7,      # 걸은 경로 길이/시간 ÷ 명령 속도 (고정 시간 시험)
    walk_max_yaw_deg=15.0,    # walk 시험에서 허용하는 방향 틀어짐
    fall_tilt_deg=60.0,       # 이 이상 기울면 넘어짐으로 간주
    stall_window=10.0,        # s   이 시간 동안 진척이 없으면 정체로 중단
    stall_progress=0.05,      # m 또는 rad
)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


# ------------------------------------------------------------------ 로봇 인터페이스
class RosRobot:
    """실제 ROS 2 연결. pose() -> (x, y, z, roll, pitch, yaw)"""

    def __init__(self, ns):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.parameter import Parameter

        rclpy.init()
        self.rclpy = rclpy
        self.Twist = Twist
        self.node = Node('gait_eval',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub = self.node.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
        self.node.create_subscription(Odometry, f'/{ns}/odom/ground_truth', self._cb, 10)
        self._pose = None
        self.ns = ns

    def _cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        r, pi, y = quat_to_rpy(q.x, q.y, q.z, q.w)
        self._pose = (p.x, p.y, p.z, r, pi, y)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def pose(self):
        return self._pose

    def send(self, vx, wz):
        m = self.Twist()
        m.linear.x, m.angular.z = float(vx), float(wz)
        self.pub.publish(m)

    def step(self):
        """시뮬레이션 시간으로 CONTROL_DT 만큼 진행"""
        import time as _t
        t_end = self.now() + CONTROL_DT
        wall_end = _t.time() + 3.0
        while self.now() < t_end:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
            if _t.time() > wall_end:
                raise RuntimeError('시뮬레이션 시간이 흐르지 않습니다. Gazebo가 떠 있고 /clock 이 나오는지 확인하세요.')

    def wait_first_pose(self, wall_timeout=20.0):
        import time as _t
        end = _t.time() + wall_timeout
        while self._pose is None and _t.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        return self._pose is not None

    def close(self):
        self.send(0.0, 0.0)
        self.node.destroy_node()
        self.rclpy.shutdown()


# ------------------------------------------------------------------ 시험 실행
def tilt_deg(pose):
    return math.degrees(max(abs(pose[3]), abs(pose[4])))


def settle(robot, seconds=2.0):
    t_end = robot.now() + seconds
    while robot.now() < t_end:
        robot.send(0.0, 0.0)
        robot.step()


def check_standing(robot):
    """시험 시작 전에 서 있는지 확인 (z 0.15~0.35, 기울기 15도 이하가 2초 유지)"""
    settle(robot, 2.0)
    p = robot.pose()
    ok = 0.15 <= p[2] <= 0.35 and tilt_deg(p) <= 15.0
    return ok, p


def run_test(robot, name):
    spec = TESTS[name]
    c = CRITERIA
    start = robot.pose()
    x0, y0, yaw0 = start[0], start[1], start[5]
    t0 = robot.now()

    yaw_acc = 0.0
    prev_yaw = yaw0
    max_tilt = tilt_deg(start)
    max_lat = 0.0
    best_progress, best_progress_t = 0.0, t0
    path_len, prev_xy = 0.0, (x0, y0)
    outcome = 'timeout'
    fell = False

    while True:
        robot.send(spec['vx'], spec['wz'])
        robot.step()
        p = robot.pose()
        t = robot.now() - t0

        yaw_acc += wrap(p[5] - prev_yaw)
        prev_yaw = p[5]
        max_tilt = max(max_tilt, tilt_deg(p))
        dx, dy = p[0] - x0, p[1] - y0
        path_len += math.hypot(p[0] - prev_xy[0], p[1] - prev_xy[1])
        prev_xy = (p[0], p[1])
        # 출발 시 진행 방향에 수직인 쏠림
        lat = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
        max_lat = max(max_lat, abs(lat))

        if tilt_deg(p) > c['fall_tilt_deg'] or p[2] < 0.06:
            fell, outcome = True, 'fell'
            break

        kind = spec['kind']
        if kind == 'distance':
            progress, done = math.hypot(dx, dy), math.hypot(dx, dy) >= spec['target']
        elif kind == 'yaw':
            progress = abs(yaw_acc)
            done = (yaw_acc >= spec['target']) if spec['target'] > 0 else (yaw_acc <= spec['target'])
        elif kind == 'x_reach':
            progress, done = p[0] - x0, p[0] >= spec['target_x']
        else:  # time
            progress, done = t, t >= spec['duration']

        if done:
            outcome = 'reached'
            break
        if kind != 'time':
            if progress > best_progress + c['stall_progress']:
                best_progress, best_progress_t = progress, robot.now()
            elif robot.now() - best_progress_t > c['stall_window']:
                outcome = 'stalled'
                break
            if t > spec['timeout']:
                outcome = 'timeout'
                break

    elapsed = robot.now() - t0
    end = robot.pose()
    disp = math.hypot(end[0] - x0, end[1] - y0)
    heading_change = math.degrees(abs(yaw_acc))

    res = dict(test=name, outcome=outcome, time_s=round(elapsed, 2),
               distance_m=round(disp, 3), lateral_max_m=round(max_lat, 3),
               yaw_change_deg=round(math.degrees(yaw_acc), 1),
               max_tilt_deg=round(max_tilt, 1),
               avg_speed=round(disp / elapsed, 3) if elapsed > 0 else 0.0,
               end_pose=[round(v, 3) for v in end[:3]], fell=fell)
    res['path_m'] = round(path_len, 3)
    # 속도비 = 실제로 "나아간" 순변위 기준 (제자리에서 흔들리며 걷는 경로 길이는 제외).
    #   직선: 순변위/시간/명령속도.  곡선(arc): 이상적인 원호의 현(chord) 길이 대비 순변위.
    res['speed_ratio'] = None
    if elapsed > 0 and spec['vx'] != 0:
        if spec['wz'] != 0 and spec['kind'] == 'time':
            th = spec['wz'] * spec['duration']
            ideal = abs(2 * (spec['vx'] / spec['wz']) * math.sin(th / 2))
            res['speed_ratio'] = round(disp / ideal, 2) if ideal > 1e-6 else None
        else:
            res['speed_ratio'] = round(disp / elapsed / abs(spec['vx']), 2)
    # 흔들림 = 경로 길이 / 순변위. 1에 가까울수록 곧게 나아감, 클수록 제자리에서 흔들리거나 헛걸음
    res['sway_ratio'] = round(path_len / disp, 1) if disp > 0.02 else None
    res['yaw_rate_ratio'] = round(yaw_acc / elapsed / spec['wz'], 2) if elapsed > 0 and spec['wz'] != 0 else None
    # 전진비 = 출발 시 방향으로 실제 나아간 거리(부호 있음) / 시간 / 명령속도. 옆으로 샌 이동은 제외된다
    fwd = (end[0] - x0) * math.cos(yaw0) + (end[1] - y0) * math.sin(yaw0)
    res['forward_m'] = round(fwd, 3)
    res['forward_ratio'] = (round(fwd / elapsed / spec['vx'], 2)
                            if elapsed > 0 and spec['vx'] != 0 and spec['wz'] == 0 else None)

    # ---- 판정
    reasons = []
    if outcome != 'reached':
        reasons.append(f'{outcome}')
    kind = spec['kind']
    tilt_limit = c['ramp_max_tilt_deg'] if name == 'ramp' else c['max_tilt_deg']
    if max_tilt > tilt_limit:
        reasons.append(f'기울기 {max_tilt:.0f}°>{tilt_limit:.0f}°')
    if name == 'straight' and outcome == 'reached':
        if res['avg_speed'] < c['straight_min_speed']:
            reasons.append(f"속도 {res['avg_speed']:.2f}<{c['straight_min_speed']}")
        if max_lat > c['straight_max_drift']:
            reasons.append(f'쏠림 {max_lat:.2f}m')
        if heading_change > c['straight_max_heading_deg']:
            reasons.append(f'방향틀어짐 {heading_change:.0f}°')
    if kind == 'yaw' and outcome == 'reached' and disp > c['turn_max_disp']:
        reasons.append(f'회전 중 이동 {disp:.2f}m')
    if kind == 'time':
        if spec['wz'] != 0:
            expected = spec['wz'] * spec['duration']
            ratio = yaw_acc / expected
            res['yaw_ratio'] = round(ratio, 2)
            lo, hi = c['arc_ratio_range']
            if not (lo <= ratio <= hi):
                reasons.append(f'회전비 {ratio:.2f}')
        if spec['vx'] != 0:
            if res['speed_ratio'] is not None and res['speed_ratio'] < c['min_speed_ratio']:
                reasons.append(f"속도비 {res['speed_ratio']:.2f}<{c['min_speed_ratio']}")
            if spec['wz'] == 0:
                if heading_change > c['walk_max_yaw_deg']:
                    reasons.append(f'방향틀어짐 {heading_change:.0f}°')
                if max_lat > c['straight_max_drift']:
                    reasons.append(f'쏠림 {max_lat:.2f}m')
        elif disp > c['turn_max_disp']:
            reasons.append(f'회전 중 이동 {disp:.2f}m')
    res['pass'] = not reasons
    res['fail_reasons'] = reasons
    return res


def run_all(robot, names, repeat=1):
    results = []
    ok, p = check_standing(robot)
    if not ok:
        print(f'[중단] 시작 전에 서 있지 않습니다 (z={p[2]:.2f}, 기울기 {tilt_deg(p):.0f}°). '
              '월드를 새로 띄운 뒤 다시 실행하세요.')
        return None
    fell_any = False
    for n in [x for _ in range(repeat) for x in names]:
        print(f'--- {n} ...', flush=True)
        r = run_test(robot, n)
        results.append(r)
        print(f"    {'PASS' if r['pass'] else 'FAIL'}  {r['outcome']}  t={r['time_s']}s  "
              f"이동={r['distance_m']}m  쏠림={r['lateral_max_m']}m  회전={r['yaw_change_deg']}°  "
              f"기울기={r['max_tilt_deg']}°  속도비={r['speed_ratio']}  전진비={r.get('forward_ratio')}  회전비={r['yaw_rate_ratio']}  "
              f"{', '.join(r['fail_reasons'])}", flush=True)
        if r['fell']:
            print('[중단] 넘어져서 남은 시험을 건너뜁니다. 월드를 새로 띄우세요.')
            break
        settle(robot, 2.0)
    return results


# ------------------------------------------------------------------ 결과 저장/비교
RESULT_DIR = os.path.expanduser('~/gait_results')


def summarize(results):
    """시험별 평균 (반복 시험 비교용)"""
    names = []
    for r in results:
        if r['test'] not in names:
            names.append(r['test'])
    print('\n시험별 요약 (평균, 반복 n회)')
    print(f"{'시험':<12}{'n':>3}{'통과':>6}{'속도비':>8}{'전진비':>8}{'회전비':>8}{'쏠림':>8}{'기울기':>8}{'흔들림':>8}")
    for n in names:
        rs = [r for r in results if r['test'] == n]

        def mean(k):
            v = [r[k] for r in rs if r.get(k) is not None]
            return f'{sum(v) / len(v):.2f}' if v else '-'
        print(f"{n:<12}{len(rs):>3}{sum(r['pass'] for r in rs):>4}/{len(rs)}"
              f"{mean('speed_ratio'):>8}{mean('forward_ratio'):>8}{mean('yaw_rate_ratio'):>8}{mean('lateral_max_m'):>8}{mean('max_tilt_deg'):>8}"
              f"{mean('sway_ratio'):>8}")


def save(results, tag, note):
    os.makedirs(RESULT_DIR, exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    path = os.path.join(RESULT_DIR, f'{stamp}_{tag}.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(dict(tag=tag, note=note, time=stamp, criteria=CRITERIA, results=results),
                  f, ensure_ascii=False, indent=2)
    return path


def list_results():
    files = sorted(glob.glob(os.path.join(RESULT_DIR, '*.json')))
    if not files:
        print(f'{RESULT_DIR} 에 저장된 결과가 없습니다.')
        return
    print(f"{'파일':<34}{'시험':<12}{'판정':<6}{'시간':>7}{'이동':>7}{'쏠림':>7}{'회전':>8}{'기울기':>7}{'속도비':>7}{'회전비':>7}")
    for fp in files:
        d = json.load(open(fp, encoding='utf-8'))
        for r in d['results']:
            print(f"{os.path.basename(fp)[:33]:<34}{r['test']:<12}{'PASS' if r['pass'] else 'FAIL':<6}"
                  f"{r['time_s']:>7}{r['distance_m']:>7}{r['lateral_max_m']:>7}"
                  f"{r['yaw_change_deg']:>8}{r['max_tilt_deg']:>7}"
                  f"{str(r.get('speed_ratio', '-')):>7}{str(r.get('yaw_rate_ratio', '-')):>7}")
        if d.get('note'):
            print(f"    메모: {d['note']}")


def main(argv=None):
    ap = argparse.ArgumentParser(description='A1 보행 평가')
    ap.add_argument('--ns', default='a1_1', help='로봇 네임스페이스 (기본 a1_1)')
    ap.add_argument('--tests', default='straight,turn_left,turn_right,arc_left,arc_right',
                    help='쉼표로 구분: ' + ','.join(TESTS))
    ap.add_argument('--tag', default='run', help='결과 파일 이름에 붙일 설정 이름 (예: baseline, mu1.0)')
    ap.add_argument('--repeat', type=int, default=1, help='시험 목록 반복 횟수 (평균 비교용, 권장 3)')
    ap.add_argument('--note', default='', help='무엇을 바꿨는지 메모')
    ap.add_argument('--list', action='store_true', help='저장된 결과 비교표 출력')
    args = ap.parse_args(argv)

    if args.list:
        list_results()
        return 0
    names = [n.strip() for n in args.tests.split(',') if n.strip()]
    bad = [n for n in names if n not in TESTS]
    if bad:
        print('알 수 없는 시험:', bad, '/ 가능:', list(TESTS))
        return 2

    robot = RosRobot(args.ns)
    try:
        if not robot.wait_first_pose():
            print(f'/{args.ns}/odom/ground_truth 가 오지 않습니다. 시뮬레이션과 로봇 스폰을 확인하세요.')
            return 1
        results = run_all(robot, names, args.repeat)
    finally:
        robot.close()
    if results is None:
        return 1
    summarize(results)
    path = save(results, args.tag, args.note)
    n_pass = sum(r['pass'] for r in results)
    print(f'\n결과: {n_pass}/{len(results)} 통과  →  {path}')
    return 0 if n_pass == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
