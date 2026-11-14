"""보행·odom 점검 (Gazebo Sim 이전용, 2026-11-10): /<ns>/cmd_vel 로 앞으로 vx, 회전 wz 를 시뮬 dur 초 -> 정답(ground_truth) 이동, CHAMP odom/raw·EKF odom 이동, 최대 기울기, 끝 높이.
사용: python3 walk_check.py <ns> <vx> <wz> <dur>   (시뮬이 떠 있어야 함; walk_check.sh 가 띄워 줌)"""
import math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
ns, vx, wz, dur = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
rclpy.init(); n = Node('walk_test', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
gt = {}; od = {}; raw = {}
def tilt(q):
    r = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)); p = math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x)))); return math.degrees(max(abs(r), abs(p)))
mx = [0.0]
def on_gt(m):
    gt.setdefault('0', m.pose.pose); gt['1'] = m.pose.pose; mx[0] = max(mx[0], tilt(m.pose.pose.orientation))
def on_od(m): od.setdefault('0', m.pose.pose); od['1'] = m.pose.pose
n.create_subscription(Odometry, f'/{ns}/odom/ground_truth', on_gt, 10)
n.create_subscription(Odometry, f'/{ns}/odom', on_od, 10)
n.create_subscription(Odometry, f'/{ns}/odom/raw', lambda m: (raw.setdefault('0', m.pose.pose), raw.__setitem__('1', m.pose.pose)), 10)
pub = n.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
w0 = time.time()
while ('0' not in gt or n.get_clock().now().nanoseconds == 0) and time.time() - w0 < 60: rclpy.spin_once(n, timeout_sec=0.1)
gt.pop('0', None); od.pop('0', None); raw.pop('0', None)
while '0' not in gt: rclpy.spin_once(n, timeout_sec=0.1)
t0 = n.get_clock().now().nanoseconds * 1e-9; w0 = time.time(); tw = Twist(); tw.linear.x = vx; tw.angular.z = wz
while n.get_clock().now().nanoseconds * 1e-9 - t0 < dur:
    pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.05)
for _ in range(20): pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
a, b = gt['0'], gt['1']
def yaw(q): return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
d = math.hypot(b.position.x-a.position.x, b.position.y-a.position.y)
dod = math.hypot(od['1'].position.x-od['0'].position.x, od['1'].position.y-od['0'].position.y) if '0' in od and '1' in od else float('nan')
draw = math.hypot(raw['1'].position.x-raw['0'].position.x, raw['1'].position.y-raw['0'].position.y) if '0' in raw and '1' in raw else float('nan')
print(f'odom/raw 이동 {draw:.2f} m')
print(f'{ns} vx={vx} wz={wz} sim {dur}s (wall {time.time()-w0:.0f}s): 정답 이동 {d:.2f} m ({d/dur:.3f} m/s, 명령의 {d/dur/max(vx,1e-9)*100 if vx else 0:.0f}%), '
      f'회전 {yaw(b.orientation)-yaw(a.orientation):+.0f} deg, odom 이동 {dod:.2f} m, 최대 기울기 {mx[0]:.1f} deg, 끝 z {b.position.z:.3f}')
