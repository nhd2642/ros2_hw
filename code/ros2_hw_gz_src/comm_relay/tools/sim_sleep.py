#!/usr/bin/env python3
"""시뮬 시간으로 N 초 기다림 (/clock 기준). 2026-11-10: 통신 시험의 COMM_OFF_AT / COMM_OFF_FOR 를 벽시계 sleep 대신.
사용: python3 sim_sleep.py 120   (시뮬이 멈춰도 끝나게 벽시계 N*20+60 s 한도)"""
import sys
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

dur = float(sys.argv[1])
rclpy.init()
n = Node('sim_sleep')
st = {}
n.create_subscription(Clock, '/clock', lambda m: st.__setitem__('t', m.clock.sec + m.clock.nanosec * 1e-9), 10)
w0 = time.monotonic()
while time.monotonic() - w0 < dur * 20 + 60:
    rclpy.spin_once(n, timeout_sec=0.2)
    if 't' in st:
        st.setdefault('t0', st['t'])
        if st['t'] - st['t0'] >= dur:
            break
rclpy.shutdown()
