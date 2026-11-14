"""같은 자리 계획 실패 -> 옮겨 보기 판단 (ROS 없음)."""
import math

from comm_relay.nudge_logic import PlanFailStreak, free_heading


def test_streak_needs_three_at_same_spot():
    s = PlanFailStreak(radius=0.1, need=3)
    assert not s.fail((0, 0)) and not s.fail((0.05, 0)) and s.fail((0.02, 0.03))
    assert not s.fail((0, 0))                       # 시작한 뒤에는 다시 0 부터


def test_streak_resets_when_moved_or_planned():
    s = PlanFailStreak(radius=0.1, need=3)
    s.fail((0, 0)); s.fail((0, 0))
    assert not s.fail((0.5, 0))                     # 움직였으면 새로 셈
    s.fail((0.5, 0)); s.ok()
    assert not s.fail((0.5, 0)) and not s.fail((0.5, 0)) and s.fail((0.5, 0))


def grid(w=40, h=40, res=0.05):
    return [0] * (w * h), w, h, res


def test_free_heading_points_away_from_wall():
    # 벽(치명 100)이 로봇 북쪽 0.25 m (y = 1.25 행 전체): 가장 트인 쪽은 남쪽 계열
    data, w, h, res = grid()
    for i in range(w):
        data[25 * w + i] = 100
    v, d = free_heading(data, w, h, res, 0.0, 0.0, 1.0, 1.0)
    assert v[1] < -0.3 and d >= 0.55


def test_free_heading_none_when_boxed_and_unknown_is_free():
    data, w, h, res = grid()
    for j in range(h):
        for i in range(w):
            if max(abs(i - 20), abs(j - 20)) == 2:   # 0.1 m 상자
                data[j * w + i] = 100
    v, d = free_heading(data, w, h, res, 0.0, 0.0, 1.025, 1.025)
    assert d < 0.15                                  # 옮겨 보기 안 함 (nav_executor 는 0.15 미만이면 시작 안 함)
    data2 = [-1] * (w * h)                           # 모름만 있으면 트인 것으로
    v2, d2 = free_heading(data2, w, h, res, 0.0, 0.0, 1.0, 1.0)
    assert v2 is not None and d2 >= 0.55 and math.isclose(math.hypot(*v2), 1.0)


def test_free_heading_corridor_does_not_point_into_wall():
    # 동서 복도 (벽 y = +-0.2 m, 조금 비대칭): 평균은 거의 0 이지만 벽을 가리키면 안 됨
    data, w, h, res = grid()
    for i in range(w):
        data[24 * w + i] = 100     # y 1.2
        data[16 * w + i] = 100     # y 0.8
    for j in range(17, 24):        # 서쪽 끝을 조금 막아 비대칭
        data[j * w + 9] = 100
    v, d = free_heading(data, w, h, res, 0.0, 0.0, 1.0, 1.0)
    assert abs(v[1]) < 0.4 and d >= 0.5      # 동쪽(또는 서쪽) 복도 방향


from comm_relay.nudge_logic import body_cmd  # noqa: E402


def test_body_cmd_never_strafes():
    # 몸이 동쪽(yaw 0): 동쪽 -> 앞으로, 서쪽 -> 뒤로, 북쪽(옆) -> 먼저 회전(왼쪽 +)
    assert body_cmd((1, 0), 0.0) == (0.08, 0.0)
    assert body_cmd((-1, 0), 0.0) == (-0.08, 0.0)
    vx, wz = body_cmd((0, 1), 0.0)
    assert vx == 0.0 and wz != 0.0
    # record8b 상황: 몸 yaw -110°(남남서), 가려는 쪽 동쪽 -> 옆 방향이라 회전부터
    vx, wz = body_cmd((1, 0), math.radians(-110))
    assert vx == 0.0 and wz != 0.0
    # 40° 틀어짐은 축 안 -> 걷기 (뒤쪽 축 포함)
    a = math.radians(140)
    assert body_cmd((math.cos(a), math.sin(a)), 0.0)[0] == -0.08


def test_body_cmd_turn_reaches_axis():
    # 회전 방향대로 돌리면 결국 걷기로 바뀜
    yaw, v = 0.0, (0.0, 1.0)
    for _ in range(200):
        vx, wz = body_cmd(v, yaw)
        if vx:
            break
        yaw += wz * 0.05
    assert vx != 0.0


from comm_relay.nudge_logic import ZeroCmdWatch


def test_zero_cmd_watch_fires_after_hold():
    z = ZeroCmdWatch(eps=0.02, hold=8.0)
    z.msg(100.0, 0.1, 0.0)                    # 마지막 움직임 t=100
    assert not z.stalled(107.9, since=90.0)
    assert z.stalled(108.0, since=90.0)


def test_zero_cmd_watch_reset_by_motion_and_since():
    z = ZeroCmdWatch(hold=8.0)
    z.msg(100.0, 0.0, 0.0)                    # 0 명령은 움직임 아님
    assert z.stalled(108.0, since=90.0)
    z.msg(105.0, 0.0, 0.3)                    # 회전도 움직임
    assert not z.stalled(108.0, since=90.0)
    assert z.stalled(113.0, since=90.0)
    assert not z.stalled(113.0, since=110.0)  # 새 추종 시작(since)부터 다시 셈


def test_zero_cmd_watch_disabled():
    z = ZeroCmdWatch(hold=0.0)
    assert not z.stalled(1e9, since=0.0)


def test_cell_cost_reads_cell_and_none_outside():
    from comm_relay.nudge_logic import cell_cost
    w, h, res = 4, 3, 0.5
    data = list(range(w * h))
    assert cell_cost(data, w, h, res, 0.0, 0.0, 1.2, 0.7) == 1 * w + 2
    assert cell_cost(data, w, h, res, 0.0, 0.0, -0.1, 0.2) is None
    assert cell_cost(data, w, h, res, 0.0, 0.0, 0.1, 1.6) is None
