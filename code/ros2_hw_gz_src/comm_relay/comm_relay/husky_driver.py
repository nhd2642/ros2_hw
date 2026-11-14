"""Husky 자율주행: 목표점(/husky/goal, 월드 좌표)까지 2D 라이다로 장애물을 피해 간다 (조향 = drive_logic.Steer).

위치 추정: 바퀴 odom 의 전진 속도를 IMU yaw 방향으로 적분 (스키드 조향은 바퀴 odom 의 회전이 부정확). 시작 자세 = spawn.
입력 : /husky/goal PoseStamped, /husky/go Bool (False 면 정지 유지 — 통신 끊김·뒤따르는 A1 기다림. 안 오면 True 로 봄),
       /husky/odom, /husky/imu, /husky/scan
출력 : /husky/cmd_vel, /husky/pose PoseStamped(world), /husky/driver_state String IDLE|GO|AVOID|BLOCKED|HOLD|ARRIVED (latched)
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String

from comm_relay.drive_logic import Steer, scan_xy

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class HuskyDriver(Node):
    def __init__(self):
        super().__init__('husky_driver')
        x, y, _ = self.declare_parameter('spawn', [0.0, 0.0, 0.0]).value
        self.lidar_x = float(self.declare_parameter('lidar_x', 0.30).value)
        # 몸 0.99 x 0.67 m: 띠 반폭 = 0.335 + 여유 0.2, 앞 = 0.52 + 여유 0.3
        self.steer = Steer(v_max=float(self.declare_parameter('v_max', 0.25).value), w_max=0.6, half=0.55, front=0.8,
                           look=2.0, tol=0.15, turn_first=0.5)
        self.x, self.y, self.yaw = float(x), float(y), None
        self.odom_t = None
        self.goal, self.go, self.pts, self.scan_t, self.state = None, True, None, 0.0, 'IDLE'
        self.create_subscription(Odometry, '/husky/odom', self.on_odom, 20)
        self.create_subscription(Imu, '/husky/imu', lambda m: setattr(self, 'yaw', yaw_of(m.orientation)), qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/husky/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/husky/goal', self.on_goal, 10)
        self.create_subscription(Bool, '/husky/go', lambda m: setattr(self, 'go', m.data), 10)
        self.pub_cmd = self.create_publisher(Twist, '/husky/cmd_vel', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/husky/pose', 10)
        self.pub_state = self.create_publisher(String, '/husky/driver_state', LATCHED)
        self.pub_state.publish(String(data=self.state))
        self.create_timer(0.1, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.odom_t is not None and self.yaw is not None and 0.0 < t - self.odom_t < 0.5:
            ds = m.twist.twist.linear.x * (t - self.odom_t)
            self.x, self.y = self.x + ds * math.cos(self.yaw), self.y + ds * math.sin(self.yaw)
        self.odom_t = t
        if self.yaw is not None:
            p = PoseStamped()
            p.header.stamp, p.header.frame_id = m.header.stamp, 'world'
            p.pose.position.x, p.pose.position.y = self.x, self.y
            p.pose.orientation.z, p.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
            self.pub_pose.publish(p)

    def on_scan(self, m):
        self.pts = scan_xy(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max, self.lidar_x)
        self.scan_t = self.now()

    def on_goal(self, m):
        g = (m.pose.position.x, m.pose.position.y)
        if g != self.goal:
            self.goal = g
            self.get_logger().info(f'Husky 목표 ({g[0]:.2f}, {g[1]:.2f})')
            self.set_state('GO')

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'Husky 주행 {self.state} -> {s}')
            self.state = s
            self.pub_state.publish(String(data=s))

    def tick(self):
        tw = Twist()
        if self.goal is None or self.state == 'ARRIVED' or self.yaw is None:
            self.pub_cmd.publish(tw)
            return
        if not self.go:
            self.set_state('HOLD')
        elif self.pts is None or self.now() - self.scan_t > 1.0:      # 라이다 없이 움직이지 않는다
            self.set_state('BLOCKED')
        else:
            dx, dy = self.goal[0] - self.x, self.goal[1] - self.y
            tw.linear.x, tw.angular.z, s = self.steer.cmd(self.pts, math.atan2(dy, dx) - self.yaw, math.hypot(dx, dy))
            self.set_state(s)
        self.pub_cmd.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    node = HuskyDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
