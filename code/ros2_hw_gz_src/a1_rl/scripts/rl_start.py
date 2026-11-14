#!/usr/bin/env python3
"""rl_sim 에 /<ns>/joy 로 상태 전환을 보낸다 (키보드 대신).
  python3 rl_start.py a1_1        # A(일어서기) -> RB+십자키 위(정책 보행) -> X(항법 모드: /<ns>/cmd_vel 따름)
  python3 rl_start.py a1_1 --no-nav   # 항법 모드는 켜지 않음"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


def main():
    ns = next((a for a in sys.argv[1:] if not a.startswith('--')), 'a1_1')
    nav = '--no-nav' not in sys.argv
    rclpy.init()
    node = Node('rl_start_' + ns)
    pub = node.create_publisher(Joy, f'/{ns}/joy', 10)
    t_end = time.time() + 60
    while pub.get_subscription_count() == 0 and time.time() < t_end:
        time.sleep(0.2)
    if pub.get_subscription_count() == 0:
        print(f'/{ns}/joy 를 구독하는 rl_sim 이 없습니다. rl_sim 이 떠 있는지 확인하세요.')
        sys.exit(1)

    def send(buttons=(), dpad_up=False):
        m = Joy()
        m.buttons = [0] * 11
        m.axes = [0.0] * 8
        for b in buttons:
            m.buttons[b] = 1
        if dpad_up:
            m.axes[7] = 1.0
        pub.publish(m)

    def press(name, buttons=(), dpad_up=False, wait=1.0, repeat=3):
        print(f'[{ns}] {name}', flush=True)
        for _ in range(repeat):
            send(buttons, dpad_up)
            time.sleep(0.02)
        send()
        time.sleep(wait)

    time.sleep(1.0)
    press('A: 일어서기 시작 (약 3~4초)', buttons=(0,), wait=8.0)
    press('RB + 십자키 위: 정책 보행 시작', buttons=(5,), dpad_up=True, wait=2.0)
    if nav:
        press('X: 항법 모드 켜기 (이후 /%s/cmd_vel 로 이동)' % ns, buttons=(2,), wait=1.0, repeat=1)   # X 는 토글이라 한 번만
    print(f'[{ns}] 완료', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
