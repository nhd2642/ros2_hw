"""사람(마네킹) 발견 -> 위치 계산 · 같은 사람 묶기 · Husky 전달 대기열 (ROS 없음, 단위 시험용). record15.

사용자 결정 (2026-11-09): 발견하면 자기 지도에 표시하고 Husky 에 알림. 통신이 끊겨 있으면 모아 두었다가 복구될 때 보냄.
발견하면 1 초 서고 탐색 계속.
위치: 카메라 사람 상자(YOLOv4-tiny, 2026-11-26 전에는 팀원 MobileNet-SSD)의 좌우 범위 -> 방위 범위, 상자 높이 -> 거리 어림, 그 방위·그 거리의 라이다 점 묶음.
"""
import math


def bearing_span(u1, u2, fx, cx, shrink=0.2):
    """상자 좌우 픽셀 (u1 < u2) -> 카메라 방위 범위 (오른쪽, 왼쪽) rad, 왼쪽(+y)이 양수. 가장자리 shrink 비율씩 줄임."""
    m = (u2 - u1) * shrink
    a, b = u1 + m, u2 - m
    return math.atan2(cx - b, fx), math.atan2(cx - a, fx)


def size_range(v1, v2, fy, img_h, height=2.0, edge=4):
    """사람 상자의 위·아래 픽셀 -> 카메라 축 방향 깊이 어림 (fy * 키 / 상자 높이). 상자가 화면 위·아래 끝에 닿으면(가까워서 잘림) None.
    height 1.75 -> 2.0 (2026-11-27): YOLOv4-tiny 상자는 몸보다 세로로 커서, 정답 자세로 잰 '깊이 x 상자 높이 / fy' 가
    1.82~2.36 (11 장 중앙값 2.0). 1.75 로는 거리를 약 15 % 짧게 어림해 라이다 확인에서 버려졌음."""
    if v1 <= edge or v2 >= img_h - edge or v2 - v1 < 8:
        return None
    return fy * height / (v2 - v1)


def locate(ranges, angle_min, inc, span, cam_dx=0.14, rmin=0.2, rmax=4.0, cluster=0.25, expect=None, tol=0.4):
    """라이다(라이다 프레임) 점 중 카메라(라이다 앞 cam_dx m)에서 본 방위가 span 안인 것에서 사람 자리를 고른다.

    expect 없음: 가장 가까운 점에서 cluster m 안 평균 (가까워서 상자가 잘린 사람 = 바로 앞).
    expect (size_range 의 깊이 어림) 있음: 깊이(카메라 축 방향 거리)가 그 값에 가장 가까운 점 묶음. tol 비율 안에 없으면 None.
      깊이로 비교 (2026-11-27): 전에는 직선거리와 비교해 시야 가장자리(방위 40°)에서 사람이 1/cos 만큼(약 30 %) 가까이
      어림된 것으로 보여 버려졌음 (세트 3 북쪽 사람: 인식 0.87~0.99 세 번 모두 '라이다 점 없음').
      왜: '가장 가까운 점'은 사람이 문 너머·물체 뒤에 있을 때 문기둥·상자에 위치를 찍어 같은 사람이 여러 명으로 보고됐고,
      rmax 4 m 밖 사람은 인식 점수 0.95 여도 버려졌다 (2026-11-12 전체 임무: 정답 9 명에 보고 14 건, 일치 7 명).
    돌려줌 (x, y, 점 수) 라이다 프레임, 없으면 None. rmin: 몸에 맞은 0.15 m 점 제외.
    """
    pts = []
    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and rmin <= r <= rmax):
            continue
        a = angle_min + i * inc
        x, y = r * math.cos(a), r * math.sin(a)
        b = math.atan2(y, x - cam_dx)
        if x > cam_dx and span[0] <= b <= span[1]:
            pts.append((math.hypot(x - cam_dx, y), x, y, x - cam_dx))       # (직선거리, x, y, 깊이)
    if not pts:
        return None
    if expect is None:
        d0 = min(pts)[0]
    else:
        p0 = min(pts, key=lambda p: abs(p[3] - expect))
        if abs(p0[3] - expect) > tol * expect:
            return None
        d0 = p0[0]
    near = [(x, y) for d, x, y, _ in pts if abs(d - d0) <= cluster]
    return sum(p[0] for p in near) / len(near), sum(p[1] for p in near) / len(near), len(near)


def apply_tf(t, x, y, z=0.0):
    """geometry_msgs/Transform 을 점에 적용."""
    q, p = t.rotation, t.translation
    # 회전 행렬 (사원수)
    xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
    xy, xz, yz, wx, wy, wz = q.x * q.y, q.x * q.z, q.y * q.z, q.w * q.x, q.w * q.y, q.w * q.z
    return (p.x + (1 - 2 * (yy + zz)) * x + 2 * (xy - wz) * y + 2 * (xz + wy) * z,
            p.y + 2 * (xy + wz) * x + (1 - 2 * (xx + zz)) * y + 2 * (yz - wx) * z)


class SurvivorBook:
    """로봇 한 대의 발견 목록 (map 좌표). merge m 안이면 같은 사람(평균 갱신), 아니면 새 사람 -> Husky 대기열."""

    def __init__(self, merge=1.0):
        self.merge = merge
        self.items = []        # dict(id, x, y, n, score, t)
        self.pending = []      # 아직 Husky 에 못 보낸 id

    def add(self, x, y, score, t):
        """(새 사람이면 True, 항목)."""
        for s in self.items:
            if math.hypot(s['x'] - x, s['y'] - y) <= self.merge:
                s['x'] = (s['x'] * s['n'] + x) / (s['n'] + 1)
                s['y'] = (s['y'] * s['n'] + y) / (s['n'] + 1)
                s['n'] += 1
                s['score'] = max(s['score'], score)
                return False, s
        s = dict(id=len(self.items) + 1, x=x, y=y, n=1, score=score, t=t)
        self.items.append(s)
        self.pending.append(s['id'])
        return True, s

    def deliver(self, link_ok):
        """통신이 되면 대기열을 전부 꺼내 돌려주고 비움, 아니면 []."""
        if not link_ok or not self.pending:
            return []
        out = [s for s in self.items if s['id'] in self.pending]
        self.pending = []
        return out


def yolo_people(rows, w, h, min_score=0.1):
    """OpenCV DNN 의 YOLO 출력 행 [cx, cy, bw, bh, objectness, 클래스 점수 ...] (0~1 정규화) -> 사람(COCO 클래스 0) 상자
    (점수, x1, y1, x2, y2) 픽셀. 점수 = 클래스 0 칸 (OpenCV 가 objectness 를 이미 곱해 둠)."""
    out = []
    for r in rows:
        s = float(r[5])
        if s >= min_score:
            cx, cy, bw, bh = r[0] * w, r[1] * h, r[2] * w, r[3] * h
            out.append((s, cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2))
    return out


def nms_1d(boxes, iou=0.5):
    """(점수, x1, y1, x2, y2) 상자들 -> 가로 구간 IoU 가 iou 넘게 겹치면 점수 높은 것만 (YOLO 의 겹친 상자 합치기)."""
    keep = []
    for b in sorted(boxes, reverse=True):
        if all(min(b[3], k[3]) - max(b[1], k[1]) <= iou * (max(b[3], k[3]) - min(b[1], k[1])) for k in keep):
            keep.append(b)
    return keep
