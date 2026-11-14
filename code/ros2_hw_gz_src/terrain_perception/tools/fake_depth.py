#!/usr/bin/env python3
"""terrain_hazards 노드 끝까지 시험용 가짜 로봇 (개발용, Gazebo 없이).
장면: test_grid.ramp_h (x 0.6~2.6 경사로, 폭 |y| <= 0.95). a1_1 이 경사로 위 (1.1, 0.45) 에서 +y(옆 가장자리)를 봄.
발행: /a1_1/odom (2D, z 0 — 실제 EKF 처럼 경사로 높이 모름), /a1_1/imu/data, /a1_1/tf (base_footprint->base_link 0.21),
      /a1_1/tf_static (base_link->terrain_camera_link 0.16/0.06/40° ->optical), /a1_1/terrain_camera/points (광학 프레임,
      카메라에서 광선을 쏜 점 = 가림 포함)."""
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage

ROOT = __file__.rsplit('/tools/', 1)[0]
sys.path[:0] = [ROOT, ROOT + '/test']
from terrain_perception.terrain_node import R_rpy, make_cloud  # noqa: E402
from test_grid import ramp_h, raycast  # noqa: E402

STATIC = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
X0, Y0, YAW, BASE_Z = 1.1, 0.45, math.pi / 2, 0.21
CAM_T, PITCH = np.array([0.16, 0.0, 0.06]), math.radians(40)


def tf(parent, child, xyz, rpy, stamp):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id, t.header.stamp = parent, child, stamp
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    t.transform.rotation.x = sr * cp * cy - cr * sp * sy
    t.transform.rotation.y = cr * sp * cy + sr * cp * sy
    t.transform.rotation.z = cr * cp * sy - sr * sp * cy
    t.transform.rotation.w = cr * cp * cy + sr * sp * sy
    return t


class Fake(Node):
    def __init__(self):
        super().__init__('fake_depth')
        self.p_tf = self.create_publisher(TFMessage, '/a1_1/tf', 10)
        self.p_st = self.create_publisher(TFMessage, '/a1_1/tf_static', STATIC)
        self.p_od = self.create_publisher(Odometry, '/a1_1/odom', 50)
        self.p_im = self.create_publisher(Imu, '/a1_1/imu/data', 50)
        self.p_c = self.create_publisher(PointCloud2, '/a1_1/terrain_camera/points', 5)
        now = self.get_clock().now().to_msg()
        self.p_st.publish(TFMessage(transforms=[
            tf('base_link', 'terrain_camera_link', CAM_T, (0, PITCH, 0), now),
            tf('terrain_camera_link', 'terrain_camera_optical', (0, 0, 0), (-math.pi / 2, 0, -math.pi / 2), now)]))
        ground = ramp_h(np.array([X0]), np.array([Y0]))[0]
        R_yaw = R_rpy(0, 0, YAW)
        cam_w = np.array([X0, Y0, ground + BASE_Z]) + R_yaw @ CAM_T
        pts_w = raycast(ramp_h, cam_w, YAW)
        # 노드의 odom 높이 = 실제 - 경사로 높이 (EKF 2D) -> base_link 좌표 -> 광학 좌표
        pb = (pts_w - np.array([X0, Y0, ground + BASE_Z])) @ R_yaw                    # = R_yaw^T (p - off)
        R_s = R_rpy(0, PITCH, 0) @ R_rpy(-math.pi / 2, 0, -math.pi / 2)
        self.opt = ((pb - CAM_T) @ R_s).astype(np.float32)                             # = R_s^T (pb - t)
        self.create_timer(0.02, self.fast)
        self.create_timer(0.1, self.cloud)
        self.k = 0

    def fast(self):
        now = self.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, 'odom', 'base_footprint'
        o.pose.pose.position.x, o.pose.pose.position.y = X0, Y0
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(YAW / 2), math.cos(YAW / 2)
        self.p_od.publish(o)
        i = Imu()
        i.header.stamp, i.orientation.w = now, 1.0
        self.p_im.publish(i)
        self.k += 1
        if self.k % 50 == 1:
            self.p_tf.publish(TFMessage(transforms=[tf('base_footprint', 'base_link', (0, 0, BASE_Z), (0, 0, 0), now)]))

    def cloud(self):
        # 실제 카메라처럼 스캔 시각이 EKF 보다 앞서게: odom 이 나중에 따라옴
        m = make_cloud('terrain_camera_optical', self.get_clock().now().to_msg(), self.opt[:, :2], 0.0)
        m.data = self.opt.tobytes()
        self.p_c.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Fake())


if __name__ == '__main__':
    main()
