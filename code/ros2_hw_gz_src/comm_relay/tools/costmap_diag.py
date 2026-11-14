"""전역 비용지도 진단 (record15): 경로 계획이 실패한 순간 로봇 둘레가 Nav2 플래너에게 어떻게 보였는지.

왜: record14 600 s 에서 a1_2 가 서쪽 방 안에서 모든 목표·원위치 계획이 0 점으로 실패(10 번)했는데 국소 비용지도는 비어
있었고 로그로는 원인을 알 수 없었음. 후보 = (a) 전역 비용지도 라이다 층에 몸에 맞은 0.15 m 점이 찍혀 로봇을 둘러쌈
(obstacle_min_range 0.2 는 국소에만 넣었음), (b) 방 출입구가 비용지도에서 닫힘.
NavFn 규칙: 비용 >= 253(내접·치명)은 못 지나감, 255(모름)는 allow_unknown 이면 지나감.
"""
from collections import deque

import numpy as np

BLOCK = 253
UNKNOWN = 255


def passable(cost):
    return (cost < BLOCK) | (cost == UNKNOWN)


def diag(cost, res, ox, oy, rx, ry, spawn=(0.0, 0.0), near=0.3):
    """cost: (h, w) uint8 비용지도, (ox, oy) 원점, (rx, ry) 로봇 map 좌표. 결과 dict.

    robot_cost: 로봇 칸 비용, near_block: near m 안 막힌 칸 수 / 전체, reach_m2: 로봇 칸(막혔으면 near 안 가장 가까운
    통과 칸)에서 4 방향으로 닿는 넓이, spawn_reach: 그 영역에 스폰(map 원점) 칸이 있는지, ring: 로봇 둘레 0.15~0.30 m
    고리에서 막힌 칸 비율 (몸에 맞은 라이다 점이면 이 고리가 거의 다 막힘).
    """
    h, w = cost.shape
    ok = passable(cost)
    c, r = int((rx - ox) / res), int((ry - oy) / res)
    out = dict(inside=0 <= r < h and 0 <= c < w)
    if not out['inside']:
        return out
    k = int(np.ceil(near / res))
    rr, cc = np.mgrid[max(0, r - k):min(h, r + k + 1), max(0, c - k):min(w, c + k + 1)]
    d = np.hypot(rr - r, cc - c) * res
    disk = d <= near
    out['robot_cost'] = int(cost[r, c])
    out['near_block'] = (int((~ok[rr, cc] & disk).sum()), int(disk.sum()))
    ring = (d >= 0.15) & (d <= 0.30)
    out['ring'] = round(float((~ok[rr, cc] & ring).sum()) / max(1, int(ring.sum())), 2)
    start = (r, c)
    if not ok[r, c]:
        cand = [(d[i, j], rr[i, j], cc[i, j]) for i, j in zip(*np.nonzero(disk & ok[rr, cc]))]
        if not cand:
            out.update(reach_m2=0.0, spawn_reach=False)
            return out
        start = min(cand)[1:]
    seen = np.zeros_like(ok)
    seen[start] = True
    q = deque([start])
    while q:
        i, j = q.popleft()
        for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= a < h and 0 <= b < w and ok[a, b] and not seen[a, b]:
                seen[a, b] = True
                q.append((a, b))
    out['reach_m2'] = round(float(seen.sum()) * res * res, 1)
    sc, sr = int((spawn[0] - ox) / res), int((spawn[1] - oy) / res)
    out['spawn_reach'] = bool(0 <= sr < h and 0 <= sc < w and seen[sr, sc])
    return out


def text(o):
    if not o.get('inside'):
        return '로봇이 비용지도 밖'
    nb = o['near_block']
    return (f"로봇 칸 비용 {o['robot_cost']}, 0.3 m 안 막힘 {nb[0]}/{nb[1]}, 0.15~0.3 m 고리 막힘 {o['ring'] * 100:.0f} %, "
            f"닿는 넓이 {o['reach_m2']} m², 스폰 닿음 {o['spawn_reach']}")
