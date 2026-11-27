"""경사로 시험 (2026-11-27): a1_1 을 세트1 남쪽 방 경사로(8°, 꼭대기 y -4.4, 북쪽 가장자리 y -3.4)에서 정해진 순서로 걷고 돌게 해
다리 odom 이 못 본 몸 이동(미끄러짐)과 SLAM 위치 오차를 단계별로 잰다. ramp_check.sh 가 시뮬·SLAM 을 띄우고 부름.
  python3 ramp_check.py <결과 폴더> [LOOPS=2]
왜: D1b 전체 1 회차에서 a1_1 이 경사로 가장자리에서 MPPI 제자리 회전 7.5 s 동안 0.67 m 밀렸는데 odom 은 거의 0 -> SLAM 1.7 m -> 4.3 m.
  전체 임무 10 회 중 1 회라 전체 임무 2 회로는 고친 안을 비교할 수 없음 -> 같은 동작을 되풀이하는 짧은 시험.
시험을 모는 쪽(이 스크립트)은 정답 자세로 방향·회전량을 맞춘다 (실행마다 같은 동작이 되게). 로봇 쪽 SLAM·odom 은 그대로.
도는 단계는 제자리 회전 (vx 0, wz 0.5).
단계 이름의 slope / flat 은 경사로 위 / 평지. 결과: ramp.csv (10 Hz), 표준 출력 = 단계별 표와 합계 (ramp.txt 로 받음)."""
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage

NS = 'a1_1'
VX, WZ = 0.35, 0.5        # MPPI vx_max·wz 한도와 같게
PI = math.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def tilt_of(q):
    r = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    p = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return math.degrees(r), math.degrees(p)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    """2D 자세 a ∘ b, 자세 = (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def sequence(loops):
    """(이름, 종류, 값) 목록. fwd: (그 방향으로 닿을 목표 좌표 — 남북이면 y, 동서면 x, 월드 방향), turn: (목표 월드 방향, 방향 부호, 덧붙일 바퀴 rad), stand: 초.
    출발 (6.4, -2.4) 남쪽 향함, 평지. 남쪽 y -3.8 = 경사로 북쪽 비탈, 거기서 서쪽 x 4.9·동쪽 x 6.4 = 비탈 옆으로 가로지르기.
    거리 대신 목표 좌표: 회전 중 미끄러짐으로 0.7 m 옮겨진 채 '북쪽 1.4 m' 를 걸으면 앞벽 문틀에 부딪혀 넘어졌음 (base_1)."""
    s = [('stand', 'stand', 5.0)]
    for _ in range(loops):
        s += [('up_slope', 'fwd', (-3.8, -PI / 2)),
              ('turn360_slope', 'turn', (-PI / 2, +1, 2 * PI)),
              ('turn_w_slope', 'turn', (PI, -1, 0.0)),
              ('cross_w_slope', 'fwd', (4.9, PI)),
              ('turn180_slope', 'turn', (0.0, +1, 0.0)),
              ('cross_e_slope', 'fwd', (6.4, 0.0)),
              ('turn_n_slope', 'turn', (PI / 2, +1, 0.0)),
              ('down_flat', 'fwd', (-2.4, PI / 2)),
              ('turn180_flat', 'turn', (-PI / 2, +1, 0.0)),
              ('stand_flat', 'stand', 5.0)]
    return s


class Check(Node):
    def __init__(self):
        super().__init__('ramp_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.gt = None          # (x, y, yaw, roll, pitch, z)
        self.map_odom = None
        self.odom_base = None
        self.create_subscription(Odometry, f'/{NS}/odom/ground_truth', self.on_gt, 20)
        self.create_subscription(TFMessage, f'/{NS}/tf', self.on_tf, 50)
        self.n_slam = 0         # slam_toolbox 가 스캔을 처리할 때마다 내는 /<ns>/pose 수 (회전 중 SLAM 이 스캔을 쓰는지)
        self.create_subscription(PoseWithCovarianceStamped, f'/{NS}/pose', lambda m: setattr(self, 'n_slam', self.n_slam + 1), 20)
        self.pub = self.create_publisher(Twist, f'/{NS}/cmd_vel', 10)

    def on_gt(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.gt = (p.x, p.y, yaw_of(q), *tilt_of(q), p.z)

    def on_tf(self, m):
        for t in m.transforms:
            pose = (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                self.map_odom = pose
            elif t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint':
                self.odom_base = pose

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def slam(self):
        return compose(self.map_odom, self.odom_base)


def main():
    out = sys.argv[1]
    loops = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    rclpy.init()
    n = Check()
    w0 = time.time()
    while (n.gt is None or n.map_odom is None or n.odom_base is None or n.now() == 0) and time.time() - w0 < 120:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.map_odom is None or n.gt is None:
        print('실패: 정답 자세나 SLAM map->odom 을 120 s 안에 못 받음')
        sys.exit(1)
    g0, s0 = n.gt[:3], n.slam()
    align = compose(g0, inverse(s0))            # SLAM 자세 -> 월드 (시작 자세로 맞춤, shared_map_merger 와 같은 방식)
    csv = open(os.path.join(out, 'ramp.csv'), 'w')
    csv.write('t,phase,vx,wz,gt_x,gt_y,gt_yaw,roll,pitch,gt_z,od_x,od_y,od_yaw,sl_x,sl_y,sl_yaw,slam_err\n')
    rows, fell = [], False
    print(f'경사로 시험 LOOPS={loops}, 출발 정답 ({g0[0]:.2f}, {g0[1]:.2f}, {math.degrees(g0[2]):.0f}°)')
    for name, kind, val in sequence(loops):
        t0, g_a, o_a = n.now(), n.gt, n.odom_base
        turned, prev_yaw = 0.0, n.gt[2]
        if kind == 'turn':
            target, sign, extra = val
            amount = (sign * (target - g_a[2])) % (2 * PI)
            if amount > 2 * PI - 0.35:              # 이미 목표 방향 (조금 지나침) -> 한 바퀴 더 돌지 않게
                amount -= 2 * PI
            amount = max(amount, 0.0) + extra
        else:
            amount = 0.0
        if kind == 'fwd':
            head = val[1]
            need = abs(val[0] - (g_a[0] if abs(math.cos(head)) > 0.5 else g_a[1]))
        limit = {'stand': lambda: val, 'fwd': lambda: need / VX * 3 + 5,
                 'turn': lambda: amount / WZ * 3 + 5}[kind]()
        err_max, tilt_max, last_log, slam0 = 0.0, 0.0, 0.0, n.n_slam
        while True:
            rclpy.spin_once(n, timeout_sec=0.02)
            t = n.now() - t0
            g = n.gt
            turned += wrap(g[2] - prev_yaw)      # 부호 있는 누적 (걸음마다 흔들리는 yaw 가 쌓이지 않게)
            prev_yaw = g[2]
            tw = Twist()
            if kind == 'stand':
                done = t >= val
            elif kind == 'fwd':
                goal, head = val
                if abs(math.cos(head)) > 0.5:          # 동서로 걸으면 x, 남북이면 y 가 목표를 지났는지
                    done = (g[0] - goal) * math.cos(head) >= 0
                else:
                    done = (g[1] - goal) * math.sin(head) >= 0
                tw.linear.x = VX
                tw.angular.z = max(-WZ, min(WZ, 1.5 * wrap(head - g[2])))
            else:
                done = sign * turned >= amount
                tw.angular.z = sign * WZ
            timeout = not done and t > limit
            if done or timeout:
                n.pub.publish(Twist())
                break
            n.pub.publish(tw)
            s = compose(align, n.slam())
            err = math.hypot(s[0] - g[0], s[1] - g[1])
            err_max = max(err_max, err)
            tilt_max = max(tilt_max, abs(g[3]), abs(g[4]))
            if g[5] < 0.12 or tilt_max > 45:
                fell = True
            if n.now() - last_log >= 0.1:
                last_log = n.now()
                o = n.odom_base
                csv.write(f'{last_log:.2f},{name},{tw.linear.x:.3f},{tw.angular.z:.3f},{g[0]:.3f},{g[1]:.3f},{g[2]:.3f},'
                          f'{g[3]:.1f},{g[4]:.1f},{g[5]:.3f},{o[0]:.3f},{o[1]:.3f},{o[2]:.3f},{s[0]:.3f},{s[1]:.3f},{s[2]:.3f},{err:.3f}\n')
            if fell:
                break
        g_b, o_b = n.gt, n.odom_base
        off = g_a[2] - o_a[2]                      # odom 이동을 월드 방향으로 돌려서 정답 이동과 비교
        dx, dy = o_b[0] - o_a[0], o_b[1] - o_a[1]
        slip = math.hypot((g_b[0] - g_a[0]) - (math.cos(off) * dx - math.sin(off) * dy),
                          (g_b[1] - g_a[1]) - (math.sin(off) * dx + math.cos(off) * dy))
        moved = math.hypot(g_b[0] - g_a[0], g_b[1] - g_a[1])
        s = compose(align, n.slam())
        err_end = math.hypot(s[0] - g_b[0], s[1] - g_b[1])
        rows.append((name, kind, n.now() - t0, moved, abs(math.degrees(turned)), slip, tilt_max, err_max, err_end, timeout,
                     n.n_slam - slam0))
        print(f'{name:14s} {n.now() - t0:5.1f} s  이동 {moved:4.2f} m  회전 {math.degrees(turned):+4.0f}°  미끄러짐 {slip:4.2f} m  '
              f'최대 기울기 {tilt_max:4.1f}°  SLAM 오차 최대 {err_max:4.2f} 끝 {err_end:4.2f} m  SLAM 처리 스캔 {n.n_slam - slam0:3d}'
              f'{"  시간 초과" if timeout else ""}', flush=True)
        if fell:
            print('넘어짐 -> 중단')
            break
    for _ in range(10):
        n.pub.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.05)
    csv.close()

    def total(cond):
        sel = [r for r in rows if cond(r)]
        return sum(r[5] for r in sel), sum(r[4] for r in sel), sum(r[3] for r in sel)
    rs, rdeg, _ = total(lambda r: r[1] == 'turn' and r[0].endswith('_slope'))
    fs, _, fm = total(lambda r: r[1] == 'fwd' and r[0].endswith('_slope'))
    ts, tdeg, _ = total(lambda r: r[1] == 'turn' and r[0].endswith('_flat'))
    print('--- 합계')
    print(f'경사 위 돌기: 미끄러짐 {rs:.2f} m / 회전 {rdeg:.0f}° = {rs / max(rdeg, 1) * 360:.2f} m/바퀴')
    print(f'평지 돌기:    미끄러짐 {ts:.2f} m / 회전 {tdeg:.0f}° = {ts / max(tdeg, 1) * 360:.2f} m/바퀴')
    print(f'경사 위 걷기: 미끄러짐 {fs:.2f} m / 이동 {fm:.2f} m = {fs / max(fm, 0.01) * 100:.0f} %')
    print(f'SLAM 오차: 최대 {max(r[7] for r in rows):.2f} m, 끝({rows[-1][0]}) {rows[-1][8]:.2f} m, '
          f'평지 멈춤 끝 값 {" / ".join(f"{r[8]:.2f}" for r in rows if r[0] == "stand_flat")}')
    print(f'넘어짐 {int(fell)}, 시간 초과 단계 {sum(1 for r in rows if r[9])}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
