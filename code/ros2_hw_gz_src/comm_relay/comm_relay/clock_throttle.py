"""/clock (1 kHz, 시뮬 스텝마다) -> /clock_slow (기본 50 Hz 시뮬 시간).

왜: CHAMP 보행은 1 kHz /clock 이 필요하지만, rclpy 노드는 /clock 을 받는 것만으로 노드마다 CPU 약 20 % 를 쓴다
(2026-11-12 임무 실행: 파이썬 노드 14 개가 각각 20 % 안팎, RTF 0.36). 파이썬 노드는 launch 에서 /clock:=/clock_slow 로 리맵한다.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


def main(args=None):
    rclpy.init(args=args)
    node = Node('clock_throttle')
    every = max(1, int(1000 / float(node.declare_parameter('rate', 50.0).value)))
    pub = node.create_publisher(Clock, '/clock_slow', 10)
    count = [0]

    def on_clock(m):
        count[0] += 1
        if count[0] % every == 0:
            pub.publish(m)

    node.create_subscription(Clock, '/clock', on_clock, 10)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
