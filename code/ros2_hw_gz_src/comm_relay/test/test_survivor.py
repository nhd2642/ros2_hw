"""comm_relay/survivor_logic.py: 사람 상자 -> 방위 -> 라이다 위치 -> map, 같은 사람 묶기, 통신 복구 때 모아 보내기."""
import math
from types import SimpleNamespace as NS

from comm_relay.survivor_logic import SurvivorBook, apply_tf, bearing_span, locate, nms_1d, size_range

W, FX, CX = 640, 320 / math.tan(0.9), 320.0     # 카메라 640 px, 수평 FOV 1.8 rad (a1.urdf.xacro)


def scan_with(objects, n=360):
    """objects: [(x, y, 반지름)] 라이다 프레임 원기둥. 360 빔, 벽 없음(inf)."""
    inc = 2 * math.pi / n
    ranges = []
    for i in range(n):
        a = -math.pi + i * inc
        best = math.inf
        for ox, oy, r in objects:
            # 광선-원 교차
            b = ox * math.cos(a) + oy * math.sin(a)
            c = ox * ox + oy * oy - r * r
            disc = b * b - c
            if disc >= 0 and b - math.sqrt(disc) > 0:
                best = min(best, b - math.sqrt(disc))
        ranges.append(best)
    return ranges, -math.pi, inc


def box_of(x, y, half_w=0.2, cam_dx=0.14):
    """라이다 프레임 (x, y) 사람(폭 2*half_w) 이 카메라에 찍히는 상자 좌우 픽셀."""
    us = [CX - FX * math.tan(math.atan2(y + s, x - cam_dx)) for s in (half_w, -half_w)]
    return min(us), max(us)


def test_bearing_span_center_and_left():
    lo, hi = bearing_span(300, 340, FX, CX, shrink=0.0)
    assert lo < 0 < hi and abs(lo + hi) < 1e-9
    lo, hi = bearing_span(0, 100, FX, CX)
    assert lo > 0 and hi > lo                       # 화면 왼쪽 = +y


def test_locate_person_ahead_left_ignores_other_objects():
    person, wall_post = (2.0, 0.8, 0.12), (1.0, -1.0, 0.1)
    ranges, amin, inc = scan_with([person, wall_post])
    u1, u2 = box_of(2.0, 0.8)
    p = locate(ranges, amin, inc, bearing_span(u1, u2, FX, CX))
    assert p is not None and math.hypot(p[0] - 2.0, p[1] - 0.8) < 0.2


def test_locate_picks_nearest_in_span_and_drops_body_hits():
    ranges, amin, inc = scan_with([(1.5, 0.0, 0.12), (3.0, 0.0, 0.5)])
    ranges[180] = 0.15                              # 몸에 맞은 점 (정면)
    u1, u2 = box_of(1.5, 0.0)
    p = locate(ranges, amin, inc, bearing_span(u1, u2, FX, CX))
    assert abs(p[0] - 1.45) < 0.15 and abs(p[1]) < 0.1


def test_locate_none_when_nothing_in_range():
    ranges, amin, inc = scan_with([(6.0, 0.0, 0.2)])
    assert locate(ranges, amin, inc, bearing_span(300, 340, FX, CX)) is None


def test_apply_tf_yaw_and_shift():
    yaw = math.pi / 2
    t = NS(translation=NS(x=1.0, y=2.0, z=0.0), rotation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))
    x, y = apply_tf(t, 1.0, 0.0)
    assert abs(x - 1.0) < 1e-9 and abs(y - 3.0) < 1e-9


def test_book_merges_same_person_and_queues_until_link():
    b = SurvivorBook(merge=1.0)
    assert b.add(2.0, 1.0, 0.9, 0.0)[0] is True
    assert b.add(2.4, 1.2, 0.95, 2.0)[0] is False   # 같은 사람 (0.45 m)
    assert b.add(5.0, 1.0, 0.8, 4.0)[0] is True
    assert b.deliver(False) == []                   # 통신 끊김: 보내지 않고 모아 둠
    out = b.deliver(True)                           # 복구: 둘 다 한 번에
    assert [s['id'] for s in out] == [1, 2] and abs(out[0]['x'] - 2.2) < 1e-9 and out[0]['score'] == 0.95
    assert b.deliver(True) == []                    # 다시 보내지 않음
    b.add(5.3, 1.0, 0.9, 6.0)                       # 이미 보낸 사람 다시 봄 -> 새 보고 없음
    assert b.deliver(True) == []


def test_nms_1d_merges_slice_duplicates_keeps_separate_people():
    # 전체 장과 조각에서 같은 사람(가로 300~360)이 두 번, 다른 사람(500~560) 한 번
    boxes = [(0.45, 300, 100, 360, 470), (0.95, 305, 90, 362, 480), (0.6, 500, 100, 560, 470)]
    keep = nms_1d(boxes)
    assert [b[0] for b in keep] == [0.95, 0.6]
    assert nms_1d([]) == []


def test_locate_with_camera_range_skips_door_post_and_keeps_far_person():
    n, inc = 360, math.radians(1.0)
    amin = -math.pi
    idx = lambda deg: int(round((math.radians(deg) - amin) / inc))
    span = (math.radians(-6), math.radians(6))
    ranges = [float('inf')] * n
    for d in range(-40, 41):                       # 5.5 m 뒤 벽
        ranges[idx(d)] = 5.5 / math.cos(math.radians(d))
    ranges[idx(0)] = ranges[idx(1)] = 5.1          # 벽 0.4 m 앞 사람 다리 (5.1 m)
    for d in range(-6, -2):                        # 그 방향 2 m 앞 문기둥 끝
        ranges[idx(d)] = 2.0
    near = locate(ranges, amin, inc, span, rmax=7.0)
    assert near is not None and abs(near[0] - 2.0) < 0.1            # 거리 어림이 없으면 문기둥에 찍힘
    far = locate(ranges, amin, inc, span, rmax=7.0, expect=5.0)
    assert far is not None and abs(far[0] - 5.2) < 0.25             # 상자 높이로 본 5 m 근처 = 사람(과 바로 뒤 벽)
    assert locate(ranges, amin, inc, span, rmax=7.0, expect=3.5) is None   # 3.5 m 근처엔 아무것도 없음 (2.0 과 5.1 모두 40 % 밖)


def test_size_range():
    fy = 254.0
    assert abs(size_range(180, 291, fy, 480) - fy * 1.75 / 111) < 1e-9     # 약 4 m
    assert size_range(0, 300, fy, 480) is None                             # 위가 잘림 = 가까움
    assert size_range(200, 479, fy, 480) is None
