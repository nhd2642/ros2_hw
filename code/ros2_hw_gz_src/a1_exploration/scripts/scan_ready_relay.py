import math
import time
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, LaserScan  # comm_relay scan_tilt_setup.sh: 몸 기울면 바닥 맞는 빔 지움
from tf2_ros import Buffer, TransformListener

class ScanReadyRelay(Node):
    def __init__(self):
        super().__init__('scan_ready_relay')
        self.buffer = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.buffer, self)
        self.queue = deque(maxlen=50)
        self.count = 0
        self.pub = self.create_publisher(
            LaserScan, '/a1_1/scan_ready', qos_profile_sensor_data)
        self.sub = self.create_subscription(
            LaserScan, '/a1_1/scan', self.scan_cb, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.process)
        # comm_relay scan_tilt_setup.sh: 리맵된 스캔 토픽(/<ns>/scan)에서 로봇 이름을 얻어 그 로봇 IMU 를 봄
        ns = self.sub.topic_name.strip('/').split('/')[0]
        self.lidar_h = float(self.declare_parameter('lidar_height', 0.28).value)
        self.imu = deque(maxlen=400)      # (시각 s, roll, pitch) rad
        self.clipped = 0
        self.create_subscription(Imu, f'/{ns}/imu/data', self.imu_cb, qos_profile_sensor_data)
        # 다른 로봇(Husky, 상대 A1)에 맞은 빔은 SLAM 에 넣지 않는다: 움직이는 로봇이 지도에 점 줄(자취)로 남아 전역 비용지도에서
        # 길을 막고 스캔 정합을 끌어당김 (2026-11-12 세트 1: Husky 를 따라가며 a1_2 지도가 번지고 위치가 2.3 m 어긋남).
        # 위치는 통신으로 받는 /husky/pose, /<상대>/pose_shared 와 자기 /<ns>/pose_shared (전부 월드). 국소 비용지도는 원래 스캔을 써서 그대로 피함
        self.me = None                    # (x, y, yaw)
        self.others = {}                  # 이름 -> (x, y, 가릴 반경 m)
        self.create_subscription(PoseStamped, f'/{ns}/pose_shared', self.me_cb, 10)
        self.create_subscription(PoseStamped, '/husky/pose', lambda m: self.others.__setitem__(
            'husky', (m.pose.position.x, m.pose.position.y, 0.75)), 10)
        for other in self.declare_parameter('robots', ['a1_1', 'a1_2']).value:
            if other != ns:
                self.create_subscription(PoseStamped, f'/{other}/pose_shared', lambda m, o=other: self.others.__setitem__(
                    o, (m.pose.position.x, m.pose.position.y, 0.5)), 10)

    def me_cb(self, m):
        q = m.pose.orientation
        self.me = (m.pose.position.x, m.pose.position.y,
                   math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def mask_robots(self, msg):
        if self.me is None or not self.others:
            return msg
        x, y, yaw = self.me
        out, n = list(msg.ranges), 0
        for ox, oy, rad in self.others.values():
            d = math.hypot(ox - x, oy - y)
            if d > msg.range_max + rad or d < 1e-3:
                continue
            half = math.asin(min(1.0, rad / d)) if d > rad else math.pi      # 그 로봇이 차지하는 방위 폭
            center = math.atan2(oy - y, ox - x) - yaw
            for i, r in enumerate(out):
                if not math.isfinite(r) or abs(r - d) > rad:
                    continue
                a = msg.angle_min + i * msg.angle_increment - center
                if abs(math.atan2(math.sin(a), math.cos(a))) <= half:
                    out[i] = float('nan')
                    n += 1
        if n:
            msg.ranges = out
        return msg

    def imu_cb(self, m):
        q = m.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        self.imu.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, roll, pitch))

    def clip_floor(self, msg):
        if not self.imu:
            return msg
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        _, roll, pitch = min(self.imu, key=lambda a: abs(a[0] - t))
        if abs(roll) < 0.02 and abs(pitch) < 0.02:
            return msg
        out = list(msg.ranges)
        n = 0
        for i, r in enumerate(out):
            if not math.isfinite(r):
                continue
            a = msg.angle_min + i * msg.angle_increment
            z = -math.sin(pitch) * math.cos(a) + math.sin(roll) * math.cos(pitch) * math.sin(a) - 0.0087
            if z < 0 and r > 0.9 * self.lidar_h / -z:
                out[i] = float('nan')
                n += 1
        if n:
            self.clipped += 1
            if self.clipped % 50 == 1:
                self.get_logger().info(f'몸 기울기(roll {math.degrees(roll):.1f}, pitch {math.degrees(pitch):.1f}°): '
                                       f'바닥 맞는 빔 {n} 개 지움 (지운 스캔 누적 {self.clipped})')
            msg.ranges = out
        return msg

    def scan_cb(self, msg):
        self.queue.append((time.monotonic(), msg))

    def process(self):
        while self.queue:
            received, msg = self.queue[0]
            stamp = Time.from_msg(msg.header.stamp)
            if self.buffer.can_transform('odom', msg.header.frame_id, stamp):
                self.pub.publish(self.clip_floor(self.mask_robots(msg)))
                self.queue.popleft()
                self.count += 1
                if self.count % 20 == 0:
                    self.get_logger().info(
                        f'Forwarded {self.count} scans, waiting={len(self.queue)}')
            elif time.monotonic() - received > 3.0:
                self.queue.popleft()
                self.get_logger().warn('Dropped scan: TF unavailable')
            else:
                break

def main():
    rclpy.init()
    node = ScanReadyRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
