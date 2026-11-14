import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from action_msgs.msg import GoalStatus


ROBOTS = ('a1_1', 'a1_2')


class FollowPathTester(Node):

    def __init__(self):
        super().__init__('follow_path_tester')

        self.action_clients = {}
        self.test_paths = {}
        self.sent = set()
        self.handles = {}
        self.started = {}
        self.cancel_requested = set()

        self.cmd_count = {ns: 0 for ns in ROBOTS}
        self.nonzero_count = {ns: 0 for ns in ROBOTS}

        self.path_subscriptions = []
        self.velocity_subscriptions = []

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        for ns in ROBOTS:

            self.action_clients[ns] = ActionClient(
                self,
                FollowPath,
                f'/{ns}/follow_path'
            )

            self.path_subscriptions.append(
                self.create_subscription(
                    Path,
                    f'/{ns}/nav2_preview_path',
                    lambda msg, robot=ns: self.on_path(robot, msg),
                    path_qos
                )
            )

            self.velocity_subscriptions.append(
                self.create_subscription(
                    Twist,
                    f'/{ns}/cmd_vel_nav_test',
                    lambda msg, robot=ns: self.on_velocity(robot, msg),
                    10
                )
            )

        self.create_timer(1.0, self.tick)

        self.get_logger().info('FollowPath Tester Started')

    def on_path(self, ns, msg):

        if ns in self.sent:
            return

        if msg.header.frame_id != 'map':
            self.get_logger().warn(
                f'{ns}: Expected path frame map'
            )
            return

        if len(msg.poses) < 2:
            self.get_logger().warn(
                f'{ns}: Path too short'
            )
            return

        if ns not in self.test_paths:
            self.get_logger().info(
                f'{ns}: Received path with {len(msg.poses)} poses'
            )

        self.test_paths[ns] = msg

    def on_velocity(self, ns, msg):

        self.cmd_count[ns] += 1

        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        moving = (
            math.hypot(vx, vy) > 0.001
            or abs(wz) > 0.001
        )

        if self.cmd_count[ns] == 1:
            self.get_logger().info(
                f'{ns}: First cmd_vel received! '
                f'vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}'
            )

        if moving:
            self.nonzero_count[ns] += 1

            if self.nonzero_count[ns] == 1:
                self.get_logger().info(
                    f'{ns}: NON-ZERO VELOCITY! '
                    f'vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}'
                )

    def tick(self):

        for ns in ROBOTS:

            if ns not in self.sent:

                if ns not in self.test_paths:
                    continue

                if not self.action_clients[ns].server_is_ready():
                    continue

                goal = FollowPath.Goal()
                goal.path = self.test_paths[ns]
                goal.controller_id = 'FollowPath'
                goal.goal_checker_id = 'general_goal_checker'

                self.sent.add(ns)

                self.get_logger().info(
                    f'{ns}: Sending FollowPath goal'
                )

                future = self.action_clients[ns].send_goal_async(goal)

                future.add_done_callback(
                    lambda f, robot=ns: self.on_accepted(robot, f)
                )

            if (
                ns in self.started
                and ns not in self.cancel_requested
                and time.monotonic() - self.started[ns] >= 8.0
            ):
                self.cancel_requested.add(ns)

                self.get_logger().info(
                    f'{ns}: Cancelling test goal'
                )

                self.handles[ns].cancel_goal_async()

    def on_accepted(self, ns, future):

        try:
            handle = future.result()

            if not handle.accepted:
                self.get_logger().warn(
                    f'{ns}: Goal rejected'
                )
                return

            self.handles[ns] = handle
            self.started[ns] = time.monotonic()

            self.get_logger().info(
                f'{ns}: FollowPath goal accepted!'
            )

            handle.get_result_async().add_done_callback(
                lambda f, robot=ns: self.on_result(robot, f)
            )

        except Exception as e:
            self.get_logger().error(f'{ns}: {e}')

    def on_result(self, ns, future):

        self.cancel_requested.add(ns)

        try:
            status = future.result().status

            status_names = {
                GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                GoalStatus.STATUS_CANCELED: 'CANCELED',
                GoalStatus.STATUS_ABORTED: 'ABORTED'
            }

            result = status_names.get(status, str(status))

            self.get_logger().info(
                f'{ns}: {result} | '
                f'cmd_vel messages={self.cmd_count[ns]} | '
                f'nonzero={self.nonzero_count[ns]}'
            )

        except Exception as e:
            self.get_logger().error(f'{ns}: {e}')


def main():

    rclpy.init()

    node = FollowPathTester()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
