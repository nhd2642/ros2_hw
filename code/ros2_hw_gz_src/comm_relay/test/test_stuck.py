"""tools/stuck_report.py: FOLLOWING 중 멈춘 시간을 보행 / Nav 원인으로 나누기."""
import csv
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from stuck_report import KINDS, MOTION_COLS, ROBOTS, analyze, classify, report  # noqa: E402


def test_classify_kinds():
    assert classify([(0.15, 0.0, 0.0)] * 10) == KINDS[0]                                   # 앞으로 가라는데 안 감
    assert classify([(0.0, 0.4, 0.0)] * 10) == KINDS[1]                                    # 돌라는데 yaw 그대로
    assert classify([(0.0, 0.4 * (-1) ** i, 0.0) for i in range(10)]) == KINDS[2]         # 좌우로 흔들림
    assert classify([(0.0, 0.4, 0.1 * i) for i in range(10)]) == KINDS[3]                  # 돌기만 함
    assert classify([(0.0, 0.0, 0.0)] * 10) == KINDS[4]                                    # 명령 0


def rows(n, state='FOLLOWING', vx=0.15, gt_step=0.0, od_step=0.0, phase='EXPLORE'):
    """a1_1 은 주어진 대로, a1_2 는 IDLE. 0.5 s 간격."""
    out = []
    for i in range(n):
        r = {'t': f'{0.5 * i:.1f}', 'phase': phase, 'set': '1'}
        vals = dict(state=state, src='nav', vx=vx, wz=0.0, gt_x=gt_step * i, gt_y=0.0, gt_yaw=0.0, od_x=od_step * i, od_y=0.0, od_yaw=0.0, vy=0.0)
        for ns in ROBOTS:
            for k in MOTION_COLS:
                r[f'{ns}_{k}'] = str(vals[k] if ns == 'a1_1' else ('IDLE' if k == 'state' else 0.0))
        out.append(r)
    return out


def test_walking_in_place_counted_as_gait_with_odom_drift():
    s = analyze(rows(41, gt_step=0.0, od_step=0.05))[(1, 'a1_1')]          # 20 s, 정답은 정지, odom 은 0.1 m/s
    assert math.isclose(s['follow'], 20.0)
    assert 14.0 <= s['stuck'] <= 16.0                                       # 처음 5 s 창이 찬 뒤부터
    assert math.isclose(s[KINDS[0]], s['stuck'])
    assert 1.4 <= s['odom_drift'] <= 1.6


def test_moving_robot_is_not_stuck_and_idle_is_ignored():
    s = analyze(rows(41, gt_step=0.075))[(1, 'a1_1')]                       # 0.15 m/s 로 실제로 감
    assert s['stuck'] == 0.0
    assert analyze(rows(41, state='IDLE')) == {}                            # 추종이 아니면 셈 안 함
    assert analyze(rows(41, phase='TRANSIT')) == {}                         # 이동 단계는 a1_follower 몫


def test_report_text(tmp_path):
    p = tmp_path / 'motion.csv'
    data = rows(41)
    with open(p, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(data[0]))
        w.writeheader()
        w.writerows(data)
    text = report(str(tmp_path))
    assert text.startswith('멈춤') and '원인 보행' in text and '  1 a1_1' in text
    assert report(str(tmp_path / 'none')).startswith('멈춤: motion.csv 없음')
