import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose


class PathPreview(Node):

    def __init__(self):

        super().__init__('nav2_path_preview')

        self.action_clients = {}
        self.path_publishers = {}
        self.goal_subscriptions = []

        self.sent = set()
        self.pending = set()

        for ns in ('a1_1', 'a1_2'):

            self.action_clients[ns] = ActionClient(
                self,
                ComputePathToPose,
                f'/{ns}/compute_path_to_pose'
            )

            self.path_publishers[ns] = self.create_publisher(
                Path,
                f'/{ns}/nav2_preview_path',
                QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL
                )
            )

            self.goal_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    f'/{ns}/nav_goal_local',
                    lambda msg, robot=ns: self.on_goal(robot, msg),
                    10
                )
            )

        self.get_logger().info(
            'Waiting for local goals and Nav2 planners...'
        )

    def on_goal(self, ns, pose):

        # 테스트를 위해 각 로봇당 한 번만 경로 요청
        if ns in self.sent or ns in self.pending:
            return

        if pose.header.frame_id != 'map':
            self.get_logger().error(
                f'{ns}: Expected map frame'
            )
            return

        if not self.action_clients[ns].server_is_ready():
            return

        self.pending.add(ns)

        request = ComputePathToPose.Goal()

        request.goal = pose
        request.planner_id = 'GridBased'
        request.use_start = False

        self.get_logger().info(
            f'{ns}: Requesting Nav2 path'
        )

        future = self.action_clients[ns].send_goal_async(request)

        future.add_done_callback(
            lambda f, robot=ns: self.on_accepted(robot, f)
        )

    def on_accepted(self, ns, future):

        try:
            handle = future.result()

            if not handle.accepted:
                self.get_logger().warn(
                    f'{ns}: Goal rejected'
                )
                self.pending.discard(ns)
                return

            handle.get_result_async().add_done_callback(
                lambda f, robot=ns: self.on_result(robot, f)
            )

        except Exception as exc:
            self.get_logger().error(f'{ns}: {exc}')
            self.pending.discard(ns)

    def on_result(self, ns, future):

        self.pending.discard(ns)
        self.sent.add(ns)

        try:
            response = future.result()

            if (
                response.status == GoalStatus.STATUS_SUCCEEDED
                and response.result.path.poses
            ):

                path = response.result.path

                self.path_publishers[ns].publish(path)

                self.get_logger().info(
                    f'{ns}: Path calculated! '
                    f'{len(path.poses)} poses'
                )

            else:
                self.get_logger().warn(
                    f'{ns}: Planning failed. '
                    f'Status={response.status}'
                )

        except Exception as exc:
            self.get_logger().error(f'{ns}: {exc}')


def main():

    rclpy.init()

    node = PathPreview()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
