#!/usr/bin/env python3
"""깊이 카메라 점구름 -> Nav2 비용지도용 위험 칸 / 빈 칸 점구름 (로봇별).

입력 (로봇마다):
  /<ns>/<cloud_topic>   sensor_msgs/PointCloud2  (기본 terrain_camera/points, Gazebo depth 카메라)
  /<ns>/odom            EKF (odom -> base_footprint 의 x, y, yaw)
  /<ns>/imu/data        몸통 roll/pitch  (state_estimation TF 는 roll/pitch 0 고정 -> 경사로 위에서는 지면 전체가
                        10° 기울어 보여 기울기 판정(3)이 틀어짐. 2026-11-08)
  /<ns>/tf_static       base_link -> 카메라 광학 프레임 (고정)
  /<ns>/tf              base_footprint -> base_link 높이 한 번만 읽고 구독 해제 (관절 TF 가 초당 수천 개라 계속
                        받으면 파이썬 노드가 밀림: 1차 버전에서 a1_1 이 22 s 동안 7 프레임만 처리)
출력 (odom 프레임, z = out_z):
  /<ns>/terrain_hazards  위험 칸 중심 (기억 중, 반경 radius 안) -> 비용지도 marking
  /<ns>/terrain_free     로봇 둘레 radius 원 위 점들            -> 비용지도 clearing: 카메라에서 이 원까지 광선이
                         원 안 지형 층을 다 지우고 바로 위험 칸이 다시 찍힘 -> 지형 층 = 이 노드의 기억 (반경 안).
                         (1차: 이번 프레임 빈 칸만 보냄 -> 카메라가 다시 안 보는 로봇 옆·뒤 표시가 안 지워져
                         로봇 둘레 0.45 m 안이 치명 126~171칸으로 막힘, 2026-11-08 탐색 A/B)
로봇 중심 self_radius(0.35 m) 안 위험은 버림: 몸 상자 끝에서 다리가 잡혀 생기는 위험(가장 가까운 0.24~0.32 m)이
  그 자리 비용지도를 막았음. 그 안은 어차피 이미 몸이 있는 곳.
판정은 grid.py (한 프레임 안 판정 + 가림 너머 낙차, 결과만 기억). 몸 상자(base_link 기준) 안 점은 버린다(자기 다리).
임무 단계 (/mission/state): 탐색 단계(EXPLORE·REGROUP)에만 처리한다. 이동 단계에 앞서가는 Husky 뒷면이 '단차 위험'으로 찍혀
  도로를 따라 위험 줄이 전역 비용지도에 남았고(지형 층은 반경 밖을 안 지움), 다음 세트에서 탐색 시작 위치로 가는 계획이 실패했음 (2026-11-12).
다른 로봇 가림: Husky(/husky/pose)와 상대 A1(/<상대>/pose_shared) 둘레 other_radius 안 칸은 기억에 넣지 않는다 (공동 건물에서 서로를 봄).
  위치는 월드 좌표라 자기 /<ns>/pose_shared 와의 차를 odom 위치에 더해 쓴다 (odom 방향 = 월드 방향).
라이다 억제: /<ns>/<lidar_topic> (기본 scan_ready = relay 가 기울 때 바닥 빔을 지운 스캔) 이 맞힌 칸 lidar_clear(0.25 m)
  안 위험은 내보내지 않음 (벽 밑동 = 라이다 몫, grid.RollingState 설명). lidar_clear 0 이면 끔.
"""
import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from mission_interfaces.msg import MissionState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

from terrain_perception.grid import FREE, RollingState, classify_frame, pad_hazards

RING = np.radians(np.arange(0, 360, 2))
STATIC = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
_DT = {PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64}


def cloud_xyz(msg):
    """PointCloud2 -> (N, 3) float64, NaN/inf 제거."""
    f = {x.name: x for x in msg.fields}
    if not all(k in f for k in 'xyz') or msg.width * msg.height == 0:
        return np.zeros((0, 3))
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': [_DT[f[k].datatype] for k in 'xyz'],
                   'offsets': [f[k].offset for k in 'xyz'], 'itemsize': msg.point_step})
    arr = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
    p = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)
    return p[np.isfinite(p).all(axis=1)]


def make_cloud(frame, stamp, xy, z):
    m = PointCloud2()
    m.header.frame_id, m.header.stamp = frame, stamp
    m.height, m.width = 1, len(xy)
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
    m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 12, 12 * len(xy), True
    pts = np.zeros((len(xy), 3), np.float32)
    if len(xy):
        pts[:, :2] = xy
        pts[:, 2] = z
    m.data = pts.tobytes()
    return m


def quat_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tf_matrix(t):
    q, p = t.transform.rotation, t.transform.translation
    return quat_R(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])


def rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def R_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def interp(hist, t):
    """hist: deque of (t, a, b, c) 시간순. t 를 감싸는 두 표본으로 선형 보간 (각도는 c 만, wrap). 없으면 None."""
    if len(hist) < 2 or t < hist[0][0] or t > hist[-1][0]:
        return None
    for k in range(len(hist) - 1, 0, -1):
        t0, t1 = hist[k - 1][0], hist[k][0]
        if t0 <= t <= t1:
            f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            a = [hist[k - 1][i] + f * (hist[k][i] - hist[k - 1][i]) for i in (1, 2)]
            dc = math.atan2(math.sin(hist[k][3] - hist[k - 1][3]), math.cos(hist[k][3] - hist[k - 1][3]))
            return a[0], a[1], hist[k - 1][3] + f * dc
    return None


class Robot:
    def __init__(self, node, ns, p):
        self.node, self.ns, self.p = node, ns, p
        self.buf = Buffer()
        self.odom = deque(maxlen=400)          # (t, x, y, yaw)   EKF 50 Hz -> 8 s
        self.imu = deque(maxlen=800)           # (t, roll, pitch, 0)
        self.base_z = None                     # base_footprint -> base_link 높이
        self.frame = None
        node.create_subscription(TFMessage, f'/{ns}/tf_static',
                                 lambda m: [self.buf.set_transform_static(t, 'terrain') for t in m.transforms], STATIC)
        self.sub_tf = node.create_subscription(TFMessage, f'/{ns}/tf', self.on_tf, 10)
        node.create_subscription(Odometry, f'/{ns}/odom', self.on_odom, 50)
        node.create_subscription(Imu, f'/{ns}/imu/data', self.on_imu, qos_profile_sensor_data)
        node.create_subscription(PointCloud2, f'/{ns}/{p["cloud_topic"]}', self.on_cloud, qos_profile_sensor_data)
        self.scan = None
        if p['lidar_clear'] > 0:
            node.create_subscription(LaserScan, f'/{ns}/{p["lidar_topic"]}', self.on_scan, qos_profile_sensor_data)
        self.pub_hz = node.create_publisher(PointCloud2, f'/{ns}/terrain_hazards', 5)
        self.pub_fr = node.create_publisher(PointCloud2, f'/{ns}/terrain_free', 5)
        self.mem = RollingState(cell=p['cell'], size=2 * p['radius'] + 4.0, forget=p['forget'])
        self.queue = deque(maxlen=3)
        self.last_proc = 0.0
        self.stats = dict(frames=0, wait=0, dropped=0, notf=0, ms=0.0, hz=0, pts=0, pitch=0.0, lid=0, sup=0, lnotf=0)

    def on_scan(self, m):
        self.scan = m

    def on_tf(self, m):
        for t in m.transforms:
            if t.child_frame_id.endswith('base_link') and t.header.frame_id.endswith('base_footprint'):
                self.base_z = t.transform.translation.z
                self.node.get_logger().info(f'[{self.ns}] base_footprint->base_link 높이 {self.base_z:.3f} m')
                self.node.destroy_subscription(self.sub_tf)
                return

    def on_odom(self, m):
        q = m.pose.pose.orientation
        self.odom.append((Time.from_msg(m.header.stamp).nanoseconds * 1e-9, m.pose.pose.position.x,
                          m.pose.pose.position.y, rpy(q)[2]))

    def on_imu(self, m):
        r, p, _ = rpy(m.orientation)
        self.imu.append((Time.from_msg(m.header.stamp).nanoseconds * 1e-9, r, p, 0.0))

    def on_cloud(self, m):
        self.queue.append(m)

    def tick(self):
        if not self.node.active:
            self.queue.clear()
            return
        if not self.queue or time.monotonic() - self.last_proc < 1.0 / self.p['rate'] or self.base_z is None:
            return
        msg = self.queue[-1]                                   # 가장 최근 것
        ts = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        frame = msg.header.frame_id if self.p['cloud_frame'] in ('', 'auto') else self.p['cloud_frame']
        od, im = interp(self.odom, ts), interp(self.imu, ts)
        if od is None or im is None:
            if self.odom and ts > self.odom[-1][0]:
                self.stats['wait'] += 1                        # EKF 가 스캔보다 0.2~0.3 s 늦음 -> 다음 tick 에
                return
            self.queue.pop()
            self.stats['dropped'] += 1
            return
        try:
            Rs, ts_ = tf_matrix(self.buf.lookup_transform(self.p['base_link'], frame, Time()))
        except Exception:
            self.stats['notf'] += 1                            # tf_static 아직 (base_link -> 카메라)
            return
        self.queue.clear()
        self.last_proc = time.monotonic()
        self.process(msg, ts, od, im, Rs, ts_)

    def process(self, msg, ts, od, im, Rs, tsv):
        t0 = time.perf_counter()
        p = self.p
        pts = cloud_xyz(msg)
        pts = pts[np.linalg.norm(pts, axis=1) <= p['max_range']]
        pb = pts @ Rs.T + tsv                                  # base_link 기준
        body = (pb[:, 0] > -p['body_back']) & (pb[:, 0] < p['body_front']) & (np.abs(pb[:, 1]) < p['body_half']) \
            & (pb[:, 2] < p['body_top'])
        pb = pb[~body]
        x, y, yaw = od
        roll, pitch, _ = im
        R = R_rpy(roll, pitch, yaw)                            # odom 방향 = 월드 방향 (EKF yaw = IMU 절대 yaw)
        off = np.array([x, y, self.base_z])
        po = pb @ R.T + off
        cam = R @ tsv + off
        ij, st = classify_frame(po, cam_xy=cam[:2], cam_z=cam[2], cell=p['cell'], min_points=p['min_points'], max_step=p['max_step'],
                                max_slope_deg=p['max_slope_deg'], bridge=p['bridge'], bridge_min_angle=p['bridge_min_angle'])
        c = (ij + 0.5) * p['cell']
        near = (c[:, 0] - x) ** 2 + (c[:, 1] - y) ** 2 < p['self_radius'] ** 2
        me = self.node.world.get(self.ns)
        for name, (wx, wy) in (self.node.world.items() if me else ()):
            if name != self.ns:           # 다른 로봇 자리 (odom 좌표로 옮겨서)
                near |= (c[:, 0] - (x + wx - me[0])) ** 2 + (c[:, 1] - (y + wy - me[1])) ** 2 < p['other_radius'] ** 2
        self.mem.update(ij[~near], st[~near], ts, x, y)
        lid = self.mark_lidar(ts, od, im)
        hz = self.mem.hazards_xy(ts, x, y, p['radius'], lidar_clear=p['lidar_clear'])
        hz = pad_hazards(hz, p['hazard_pad'], p['cell'])   # fix16: 위험에서 벽보다 더 떨어지게
        fr = np.stack([x + p['radius'] * np.cos(RING), y + p['radius'] * np.sin(RING)], 1)
        self.pub_hz.publish(make_cloud(p['odom_frame'], msg.header.stamp, hz, p['out_z']))
        self.pub_fr.publish(make_cloud(p['odom_frame'], msg.header.stamp, fr, p['out_z']))
        s = self.stats
        s['frames'] += 1
        s['ms'] += (time.perf_counter() - t0) * 1000
        s['hz'], s['pts'], s['pitch'] = len(hz), len(po), math.degrees(pitch)
        s['lid'] += lid
        s['sup'] = self.mem.suppressed

    def mark_lidar(self, ts, od, im):
        """최근 스캔(구름 시각과 lidar_max_age 안)의 맞은 점을 같은 odom 격자에 기록. 기록했으면 1."""
        p, m = self.p, self.scan
        if m is None:
            return 0
        tl = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        if abs(tl - ts) > p['lidar_max_age']:
            return 0
        try:
            Rl, tvl = tf_matrix(self.buf.lookup_transform(p['base_link'], m.header.frame_id, Time()))
        except Exception:
            self.stats['lnotf'] += 1                           # base_link -> 라이다 고정 TF 없음
            return 0
        r = np.asarray(m.ranges, np.float64)
        a = m.angle_min + m.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r >= max(p['lidar_min_range'], m.range_min)) & (r <= min(p['radius'] + 0.5, m.range_max))
        pl = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum())], 1)
        x, y, yaw = interp(self.odom, tl) or od                 # 스캔 시각 자세 (없으면 구름 시각)
        roll, pitch, _ = interp(self.imu, tl) or im
        po = (pl @ Rl.T + tvl) @ R_rpy(roll, pitch, yaw).T + np.array([x, y, self.base_z])
        self.mem.mark_lidar(po[:, :2], ts, od[0], od[1])
        return 1


class TerrainNode(Node):
    def __init__(self):
        super().__init__('terrain_hazards')
        robots = list(self.declare_parameter('robots', ['a1_1', 'a1_2']).value)
        defaults = dict(cloud_topic='terrain_camera/points', cloud_frame='auto', odom_frame='odom', base_link='base_link',
                        cell=0.05, max_step=0.04, max_slope_deg=25.0, bridge=0.6, bridge_min_angle=25.0, min_points=2, max_range=2.0,
                        radius=3.0, forget=60.0, rate=5.0, out_z=0.1, self_radius=0.35,
                        body_front=0.42, body_back=0.40, body_half=0.25, body_top=0.30,
                        lidar_topic='scan_ready', lidar_clear=0.25, lidar_min_range=0.25, lidar_max_age=0.5, hazard_pad=0.0, other_radius=0.8)
        self.p = {k: type(v)(self.declare_parameter(k, v).value) for k, v in defaults.items()}
        self.active = False                    # 임무가 탐색 단계인가
        self.world = {}                        # 이름 -> 월드 (x, y): husky, a1_1, a1_2
        self.create_subscription(MissionState, '/mission/state', lambda m: setattr(
            self, 'active', m.phase in ('EXPLORE', 'REGROUP')), QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for name, topic in [('husky', '/husky/pose')] + [(ns, f'/{ns}/pose_shared') for ns in robots]:
            self.create_subscription(PoseStamped, topic, lambda m, n=name: self.world.__setitem__(
                n, (m.pose.position.x, m.pose.position.y)), 10)
        self.robots = [Robot(self, ns, self.p) for ns in robots]
        self.create_timer(0.05, lambda: [r.tick() for r in self.robots])
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'terrain_hazards: robots={robots}, params={self.p}')

    def report(self):
        parts = []
        for r in self.robots:
            s = r.stats
            n = max(s['frames'], 1)
            parts.append(f'{r.ns} 처리 {s["frames"]} (odom 대기 {s["wait"]}, 버림 {s["dropped"]}, 고정 TF 없음 {s["notf"]}), '
                         f'평균 {s["ms"] / n:.0f} ms, '
                         f'점 {s["pts"]}, 위험 칸 {s["hz"]} (라이다 억제 {s["sup"]}, 라이다 반영 {s["lid"]}, 라이다 TF 없음 {s["lnotf"]}), 몸통 pitch {s["pitch"]:+.1f}°'
                         + ('' if r.base_z is not None else ', base_link 높이 아직'))
            r.stats = dict(frames=0, wait=0, dropped=0, notf=0, ms=0.0, hz=s['hz'], pts=s['pts'], pitch=s['pitch'], lid=0, sup=s['sup'], lnotf=0)
        self.get_logger().info(' | '.join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = TerrainNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
