#!/usr/bin/env python3
"""멈춤 분석 (2026-11-19): mission_watch.py 가 2 Hz 로 쓴 motion.csv 에서 'FOLLOWING 인데 몸이 안 가는' 시간을 원인별로 나눈다.

  python3 stuck_report.py <결과 폴더 또는 motion.csv>      -> 표준 출력 (mission_watch 는 같은 내용을 stuck.txt 로)

왜: 실행마다 DWB 'Failed to make progress' 가 6~32 번인데, 그때 보행이 명령을 못 따른 건지(제자리걸음) DWB 가 머뭇거린 건지
  기존 기록(10 s 표본, 로그)으로는 못 가렸다. 고칠 곳(보행 / Nav / SLAM)을 고르려면 이 비율이 먼저 필요하다.
멈춤 = 탐색 단계(EXPLORE·REGROUP)에서 실행기가 FOLLOWING 으로 WINDOW 초 이어졌는데 그동안 정답 위치가 STILL m 도 안 움직임.
  그 표본 하나(0.5 s)를 그 창의 명령으로 분류한다 (먼저 맞는 것):
  - 보행: 전진 안 됨  전진 명령이 이어짐 (|vx| 평균 >= FWD, 0.03 넘는 표본 60 % 이상) -> 몸이 명령을 못 따름
  - Nav: 진동         vx 나 wz 부호가 FLIPS 번 이상 바뀜 -> 플래너가 좌우·앞뒤로 흔들림
  - 보행: 회전 안 됨  회전 명령(|wz| 평균 >= TURN)인데 정답 yaw 가 YAW_STILL rad 도 안 바뀜
  - Nav: 제자리 회전  회전 명령대로 몸은 도는데 진전 없음 (돌기만 함)
  - Nav: 명령 0       그 밖 (명령이 거의 0)
  멈춘 동안 odom 이 움직인 거리(정답은 정지) = 'odom 흐름': 제자리걸음이 SLAM 위치를 끄는 양.
"""
import csv
import math
import os
import sys
from collections import deque

MOTION_COLS = ('state', 'src', 'vx', 'wz', 'gt_x', 'gt_y', 'gt_yaw', 'od_x', 'od_y', 'od_yaw')
ROBOTS = ('a1_1', 'a1_2')
WINDOW, STILL, FWD, TURN, YAW_STILL, FLIPS = 5.0, 0.10, 0.05, 0.15, 0.2, 3
KINDS = ('보행: 전진 안 됨', '보행: 회전 안 됨', 'Nav: 진동', 'Nav: 제자리 회전', 'Nav: 명령 0')


def flips(vals, eps):
    """eps 보다 큰 값만 보고 부호가 바뀐 횟수."""
    signs = [v > 0 for v in vals if abs(v) > eps]
    return sum(1 for a, b in zip(signs, signs[1:]) if a != b)


def yaw_change(yaws):
    """연속 yaw 들의 누적 회전량 (rad, 절댓값 합)."""
    return sum(abs(math.atan2(math.sin(b - a), math.cos(b - a))) for a, b in zip(yaws, yaws[1:]))


def classify(win):
    """win = [(vx, wz, gt_yaw), ...] (멈춘 창) -> KINDS 중 하나."""
    vx = [w[0] for w in win]
    wz = [w[1] for w in win]
    n = len(win)
    if sum(abs(v) for v in vx) / n >= FWD and sum(1 for v in vx if abs(v) >= 0.03) >= 0.6 * n:
        return KINDS[0]
    if flips(vx, 0.02) + flips(wz, 0.05) >= FLIPS:
        return KINDS[2]
    if sum(abs(w) for w in wz) / n >= TURN:
        return KINDS[3] if yaw_change([w[2] for w in win]) >= YAW_STILL else KINDS[1]
    return KINDS[4]


def load(path):
    if os.path.isdir(path):
        path = os.path.join(path, 'motion.csv')
    try:
        with open(path, newline='') as f:
            return list(csv.DictReader(f))
    except OSError:
        return None


def analyze(rows):
    """{(세트, 로봇): {'follow': s, 'stuck': s, 종류: s, ..., 'odom_drift': m}}."""
    out = {}
    for ns in ROBOTS:
        win, prev_t, prev_od = deque(), None, None
        for r in rows:
            try:
                t = float(r['t'])
                vx, wz = float(r[f'{ns}_vx']), float(r[f'{ns}_wz'])
                gx, gy, gyaw = float(r[f'{ns}_gt_x']), float(r[f'{ns}_gt_y']), float(r[f'{ns}_gt_yaw'])
                od = (float(r[f'{ns}_od_x']), float(r[f'{ns}_od_y']))
            except (KeyError, ValueError):
                continue
            dt = min(1.0, t - prev_t) if prev_t is not None else 0.0
            prev_t = t
            following = r['phase'] in ('EXPLORE', 'REGROUP') and r[f'{ns}_state'] == 'FOLLOWING'
            if not following:
                win.clear()
                prev_od = od
                continue
            s = out.setdefault((int(r['set']), ns), dict({k: 0.0 for k in KINDS}, follow=0.0, stuck=0.0, odom_drift=0.0))
            s['follow'] += dt
            win.append((t, vx, wz, gx, gy, gyaw))
            while win and t - win[0][0] > WINDOW:
                win.popleft()
            if t - win[0][0] >= WINDOW - 0.6 and math.hypot(gx - win[0][3], gy - win[0][4]) < STILL:
                s['stuck'] += dt
                s[classify([(w[1], w[2], w[5]) for w in win])] += dt
                if prev_od is not None and not math.isnan(od[0] + prev_od[0]):
                    s['odom_drift'] += math.hypot(od[0] - prev_od[0], od[1] - prev_od[1])
            prev_od = od
    return out


def report(path):
    rows = load(path)
    if not rows:
        return '멈춤: motion.csv 없음 또는 비어 있음'
    res = analyze(rows)
    tot = {ns: dict({k: 0.0 for k in KINDS}, follow=0.0, stuck=0.0, odom_drift=0.0) for ns in ROBOTS}
    for (_, ns), s in res.items():
        for k, v in s.items():
            tot[ns][k] += v
    gait = sum(t[KINDS[0]] + t[KINDS[1]] for t in tot.values())
    nav = sum(t[k] for t in tot.values() for k in KINDS[2:])
    head = ('멈춤 (FOLLOWING 중 정답 5 s 동안 0.1 m 미만): ' + ', '.join(
        f"{ns} {t['stuck']:.0f}/{t['follow']:.0f} s ({100 * t['stuck'] / t['follow'] if t['follow'] else 0:.0f} %), odom 흐름 {t['odom_drift']:.2f} m"
        for ns, t in tot.items()) + f'; 원인 보행 {gait:.0f} s / Nav {nav:.0f} s')
    lines = [head, '', '세트 로봇   추종 s  멈춤 s  ' + '  '.join(KINDS) + '  odom흐름 m']
    for (k, ns), s in sorted(res.items()):
        lines.append(f"{k:>3} {ns}  {s['follow']:6.0f}  {s['stuck']:6.0f}  " +
                     '  '.join(f'{s[kind]:{len(kind)}.0f}' for kind in KINDS) + f"  {s['odom_drift']:8.2f}")
    return '\n'.join(lines)


if __name__ == '__main__':
    print(report(sys.argv[1] if len(sys.argv) > 1 else '.'))
