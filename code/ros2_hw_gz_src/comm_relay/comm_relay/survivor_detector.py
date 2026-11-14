"""사람(마네킹) 발견 노드 (record15). 로봇마다: 카메라 사람 인식 -> 라이다로 거리 -> 자기 map 좌표 -> 지도 표시·1 초 정지·Husky 알림.

인식은 팀원 detect_human/src/mannequin_test/scripts/a1_person_camera_ai.py 와 같은 방식:
  OpenCV DNN MobileNet-SSD (Caffe, PASCAL VOC person = 15, CPU, 300x300, scale 0.007843, mean 127.5), 로봇마다 period s(기본 1 s, record17) 에 1 장.
  2026-11-10: 전체 장 + 가로로 겹친 3 조각(폭 1/2, 1/4 씩 이동)을 따로 추론해 원래 좌표로 모은 뒤 가로 구간 IoU 0.5 NMS
  (SAHI 방식, Akyon et al. 2022). 640x480 을 300x300 으로 줄이면 멀리 작게 보이거나 다리만 보이는 사람을 놓침 (SSD 는 작은 물체에 약함,
  Liu et al. 2016) -> 클라우드 점검에서 사람이 보이는 11 장 중 conf 0.4 넘는 장 1 -> 5, 사람 없는 장 421 개에서 오탐 0 (전과 같음).
  모델 파일 model_dir (기본 <작업 공간>/person_ai_models/MobileNet-SSD: deploy.prototxt, mobilenet_iter_73000.caffemodel).
  모델이나 cv2 가 없으면 경고만 내고 아무것도 안 함 (탐색은 그대로).
입력 (로봇별): /<ns>/front_camera/image_raw, /<ns>/front_camera/camera_info, /<ns>/scan, /<ns>/tf(+_static), /<ns>/comm_status
출력 (로봇별):
  /<ns>/survivor_markers  visualization_msgs/MarkerArray (map, latched)  — 자기 지도 위 발견 위치 (RViz Fixed Frame map)
  /<ns>/survivor          geometry_msgs/PointStamped (map)              — 새 사람 발견할 때마다
  /<ns>/survivor_events   std_msgs/String  '발견 #1 x y 점수' / '전달 #1 x y 지연 s'
  /<ns>/person_score      std_msgs/Float32 — 추론한 장마다 가장 높은 사람 점수 (0 = 없음; 기준 conf 미만도) — record16 진단
  환경 변수 SURVIVOR_FRAMES_DIR 가 있으면 추론한 장을 절반 크기 jpg 로 저장 (로봇당 최대 600 장, 상자·점수 표시) — record16 진단
  /<ns>/cmd_vel_hold      geometry_msgs/Twist 0 을 hold s(시뮬 시간) 동안 20 Hz — cmd_vel_arbiter 에서 return 보다 우선 -> 1 초 정지
  /husky/survivor_report  std_msgs/String JSON {robot,id,x,y,score,delay} — 통신 될 때만. 끊겨 있으면 모았다가 복구되면 보냄
  conf 0.4 -> 0.5 (2026-11-12): 임무 v2 실행에서 사람 9 명의 발견 점수는 0.89~1.00, 오탐 1 건(기둥 옆)은 0.41.
    (통신 = Husky 스위치 하트비트 /<ns>/comm_status 가 comm_timeout s 안에 옴; return_relay 와 같은 기준)
"""
import json
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, Float32, String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from comm_relay.survivor_logic import SurvivorBook, apply_tf, bearing_span, locate, nms_1d, size_range

def sim_now(node):
    """노드 시계(초). use_sim_time 이면 시뮬 시간 — 2026-11-10 벽시계(time.monotonic) 대신 (RTF 0.1~0.3 에서 한도가 시뮬 기준 3~7 배 엄격했음)"""
    return node.get_clock().now().nanoseconds * 1e-9


LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
PERSON = 15


def detect_people(net, cv2, frame):
    """전체 장 + 겹친 3 조각에서 사람 상자 (점수, x1, y1, x2, y2, 픽셀) — NMS 전. 조각 = 폭 w/2, x 시작 0, w/4, w/2."""
    h, w = frame.shape[:2]
    out = []
    for x0, ww in ((0, w), (0, w // 2), (w // 4, w // 2), (w // 2, w // 2)):
        net.setInput(cv2.dnn.blobFromImage(frame[:, x0:x0 + ww], 0.007843, (300, 300), 127.5))
        for d in net.forward()[0, 0]:
            if int(d[1]) == PERSON:
                out.append((float(d[2]), x0 + d[3] * ww, d[4] * h, x0 + d[5] * ww, d[6] * h))
    return out


def decode_bgr(msg, cv2):
    """sensor_msgs/Image -> BGR (팀원 코드와 같음, 줄 끝 여백 step 처리)."""
    enc = msg.encoding.lower()
    ch = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(enc)
    if ch is None:
        raise ValueError(f'지원 안 하는 인코딩 {msg.encoding!r}')
    px = np.frombuffer(msg.data, dtype=np.uint8)[:msg.height * msg.step].reshape(msg.height, msg.step)
    px = px[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    conv = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR, 'bgra8': cv2.COLOR_BGRA2BGR, 'mono8': cv2.COLOR_GRAY2BGR}
    return cv2.cvtColor(px, conv[enc]) if enc in conv else px.copy()


class RobotEye:
    def __init__(self, node, ns):
        import tf2_ros
        self.node, self.ns = node, ns
        self.tf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        node.create_subscription(TFMessage, f'/{ns}/tf', lambda m: self.on_tf(m, False), 50)
        node.create_subscription(TFMessage, f'/{ns}/tf_static', lambda m: self.on_tf(m, True), QoSProfile(
            depth=20, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(Image, f'/{ns}/front_camera/image_raw', self.on_image, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f'/{ns}/front_camera/camera_info',
                                 lambda m: setattr(self, 'info', m), qos_profile_sensor_data)
        node.create_subscription(LaserScan, f'/{ns}/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        node.create_subscription(Bool, f'/{ns}/comm_status', lambda m: setattr(self, 'hb', sim_now(self.node)) if m.data else None, 10)
        self.pub_marks = node.create_publisher(MarkerArray, f'/{ns}/survivor_markers', LATCHED)
        self.pub_point = node.create_publisher(PointStamped, f'/{ns}/survivor', 10)
        self.pub_event = node.create_publisher(String, f'/{ns}/survivor_events', 10)
        self.pub_hold = node.create_publisher(Twist, f'/{ns}/cmd_vel_hold', 10)
        self.pub_score = node.create_publisher(Float32, f'/{ns}/person_score', 10)
        self.frames = 0
        self.info = self.scan = None
        self.hb = None
        self.last_run = 0.0
        self.hold_until = None
        self.book = SurvivorBook(merge=node.p['merge_dist'])

    def on_tf(self, m, static):
        for t in m.transforms:
            try:
                if static:
                    self.tf.set_transform_static(t, 'survivor')
                else:
                    self.tf.set_transform(t, 'survivor')
            except Exception:
                pass

    def link_ok(self):
        return self.hb is not None and sim_now(self.node) - self.hb <= self.node.p['comm_timeout']

    def on_image(self, msg):
        n = self.node
        now = sim_now(self.node)
        if n.net is None or now - self.last_run < n.p['period']:
            return
        self.last_run = now
        try:
            frame = decode_bgr(msg, n.cv2)
        except (ValueError, n.cv2.error) as e:
            n.get_logger().warn(f'[{self.ns}] 영상 변환 실패: {e}', throttle_duration_sec=10.0)
            return
        h, w = frame.shape[:2]
        people = nms_1d(detect_people(n.net, n.cv2, frame))
        best = max((d[0] for d in people), default=0.0)
        self.pub_score.publish(Float32(data=best))
        if n.frames_dir and self.frames < 600:      # 진단: 놓친 사람이 화면에 어떻게 보였는지 (record16)
            self.frames += 1
            img = frame.copy()
            for sc, x1, y1, x2, y2 in people:
                if sc >= 0.1:
                    n.cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    n.cv2.putText(img, f'{sc:.2f}', (int(x1), max(15, int(y1) - 5)), n.cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            sim = n.get_clock().now().nanoseconds * 1e-9
            n.cv2.imwrite(os.path.join(n.frames_dir, f'{self.ns}_{sim:07.1f}_{best:.2f}.jpg'),
                          n.cv2.resize(img, (w // 2, h // 2)), [n.cv2.IMWRITE_JPEG_QUALITY, 70])
        for score, u1, v1, u2, v2 in sorted((d for d in people if d[0] >= n.p['conf']), reverse=True):
            self.place(score, max(0.0, min(u1, u2)), min(w - 1.0, max(u1, u2)), w, min(v1, v2), max(v1, v2), h)

    def place(self, score, u1, u2, w, v1, v2, h):
        n, s = self.node, self.scan
        if s is None:
            return
        if self.info is not None and self.info.k[0] > 0:
            fx, cx, fy = self.info.k[0], self.info.k[2], self.info.k[4] or self.info.k[0]
        else:
            fx, cx = (w / 2) / math.tan(n.p['hfov'] / 2), w / 2
            fy = fx
        expect = size_range(v1, v2, fy, h)          # 상자 높이로 본 거리 (가까워서 잘렸으면 None -> 가장 가까운 점)
        pt = locate(s.ranges, s.angle_min, s.angle_increment, bearing_span(u1, u2, fx, cx),
                    cam_dx=n.p['cam_dx'], rmax=n.p['max_range'], expect=expect)
        if pt is None:
            n.get_logger().info(f'[{self.ns}] 사람 인식 {score:.2f} 이지만 그 방향' + (f' {expect:.1f} m 근처' if expect else '')
                                + ' 라이다 점 없음 -> 무시', throttle_duration_sec=5.0)
            return
        try:
            t = self.tf.lookup_transform('map', s.header.frame_id, Time()).transform
        except Exception as e:
            n.get_logger().warn(f'[{self.ns}] map <- {s.header.frame_id} TF 없음: {e}', throttle_duration_sec=10.0)
            return
        mx, my = apply_tf(t, pt[0], pt[1])
        new, item = self.book.add(mx, my, score, sim_now(self.node))
        if not new:
            return
        txt = f"발견 #{item['id']} {mx:.2f} {my:.2f} 점수 {score:.2f} (라이다 {pt[2]}점, 거리 {math.hypot(pt[0], pt[1]):.2f} m)"
        n.get_logger().warn(f'[SURVIVOR] [{self.ns}] {txt}')
        self.pub_event.publish(String(data=txt))
        ps = PointStamped()
        ps.header.frame_id, ps.header.stamp = 'map', n.get_clock().now().to_msg()
        ps.point.x, ps.point.y = mx, my
        self.pub_point.publish(ps)
        self.publish_markers()
        self.hold_until = n.get_clock().now() + Duration(seconds=n.p['hold'])

    def publish_markers(self):
        arr = MarkerArray()
        stamp = self.node.get_clock().now().to_msg()
        for s in self.book.items:
            for k, (typ, z, sc) in enumerate(((Marker.CYLINDER, 0.5, (0.4, 0.4, 1.0)), (Marker.TEXT_VIEW_FACING, 1.3, (0.3, 0.3, 0.3)))):
                m = Marker()
                m.header.frame_id, m.header.stamp = 'map', stamp
                m.ns, m.id, m.type, m.action = f'survivor_{self.ns}', s['id'] * 2 + k, typ, Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = s['x'], s['y'], z
                m.pose.orientation.w = 1.0
                m.scale.x, m.scale.y, m.scale.z = sc
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.1, 0.1, 0.9
                m.text = f"사람 #{s['id']}"
                arr.markers.append(m)
        self.pub_marks.publish(arr)

    def tick(self):
        n = self.node
        if self.hold_until is not None:
            if n.get_clock().now() < self.hold_until:
                self.pub_hold.publish(Twist())
            else:
                self.hold_until = None
        for s in self.book.deliver(self.link_ok()):
            delay = sim_now(self.node) - s['t']
            n.pub_report.publish(String(data=json.dumps(dict(robot=self.ns, id=s['id'], x=round(s['x'], 2), y=round(s['y'], 2),
                                                             score=round(s['score'], 2), delay=round(delay, 1)))))
            txt = f"전달 #{s['id']} {s['x']:.2f} {s['y']:.2f} 지연 {delay:.1f} s"
            n.get_logger().warn(f'[SURVIVOR] [{self.ns}] Husky 에 {txt}')
            self.pub_event.publish(String(data=txt))


def _ws():
    """작업 공간 (<ws>/install/comm_relay/share/comm_relay 의 4 단계 위). 모델 파일 기본 위치 = <ws>/person_ai_models."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.abspath(os.path.join(get_package_share_directory('comm_relay'), '..', '..', '..', '..'))


class SurvivorDetector(Node):
    def __init__(self):
        super().__init__('survivor_detector')
        robots = list(self.declare_parameter('robots', ['a1_1', 'a1_2']).value)
        defaults = dict(model_dir=os.path.join(_ws(), 'person_ai_models', 'MobileNet-SSD'), conf=0.5, period=1.0,
                        hfov=1.8, cam_dx=0.29, max_range=7.0, merge_dist=1.0, hold=1.0, comm_timeout=1.0)
        self.p = {k: type(v)(self.declare_parameter(k, v).value) for k, v in defaults.items()}
        self.cv2 = self.net = None
        proto = os.path.join(self.p['model_dir'], 'deploy.prototxt')
        weights = os.path.join(self.p['model_dir'], 'mobilenet_iter_73000.caffemodel')
        try:
            import cv2
            cv2.setNumThreads(1)
            self.cv2 = cv2
            self.net = cv2.dnn.readNetFromCaffe(proto, weights)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        except Exception as e:
            self.net = None
            self.get_logger().error(f'사람 인식 꺼짐 (cv2 또는 모델 {weights} 없음): {e!r}')
        self.pub_report = self.create_publisher(String, '/husky/survivor_report', 10)
        self.frames_dir = os.environ.get('SURVIVOR_FRAMES_DIR', '')
        if self.frames_dir:
            os.makedirs(self.frames_dir, exist_ok=True)
        self.eyes = [RobotEye(self, ns) for ns in robots]
        self.create_timer(0.05, lambda: [e.tick() for e in self.eyes])
        self.get_logger().info(f'survivor_detector: robots={robots}, 인식 {"켬" if self.net else "꺼짐"}, params={self.p}')


def main(args=None):
    rclpy.init(args=args)
    node = SurvivorDetector()
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
