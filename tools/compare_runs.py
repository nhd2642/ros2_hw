#!/usr/bin/env python3
"""실행 비교표 (2026-11-11, 실험 규칙: 같은 시뮬 시점 비교 + 정답 면적 대비 탐색률).

정답(GT) = 월드 파일의 벽 상자 + Husky(원점, 0.99 x 0.67 m) + 마네킹(0.5 x 0.4 m)을 0.05 m 격자에 찍고,
스폰에서 4 방향으로 닿는 빈 칸 중 자기 구역(남 y<0 / 북 y>0) 칸. 탐색률 = 그 칸 중 공유 지도에서 알려진(-1 아님) 칸 비율.
공유 지도(progress/map_XXXXs.npz)는 월드 정렬 좌표(map_info = 해상도, 원점 x, y).
나머지 지표는 events.csv 의 sim(미션 기준 시뮬 초)으로 CUT 초까지만 센다 -> 900 s 실행도 600 s 실행과 같은 조건.

사용: python3 compare_runs.py <결과 폴더> <월드 파일> [실행 폴더 이름 ...]   (이름 생략 = runs.csv 의 모든 실행)
출력: 표준 출력 표 + <결과 폴더>/compare.csv
"""
import csv
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np

RES = 0.05
CUTS = (300, 600)
SPAWN = {'a1_1': (0.0, -1.1), 'a1_2': (0.0, 1.1)}
HUSKY = (0.0, 0.0, 0.99, 0.67)          # x, y, 길이, 폭
PERSON = (0.5, 0.4)
BUILDING = (4.85, 6.5)                   # 바깥벽 안쪽 반폭 x, y (벽 중심 4.925 / 6.585)
PERSON_OK = 0.6                          # 발견 위치가 마네킹에서 이 거리 안이면 '맞음'


def boxes_from_world(path):
    """(cx, cy, sx, sy, yaw) 목록: 정적 모델의 상자 충돌체 (바닥·경사로 제외: 높이 0.3 m 미만) + 마네킹."""
    w = ET.parse(path).getroot().find('world')
    out, people = [], {}
    for m in w.findall('model'):
        name = m.get('name')
        mp = [float(v) for v in (m.findtext('pose') or '0 0 0 0 0 0').split()]
        if name.startswith('person'):
            people[name] = (mp[0], mp[1])
            out.append((mp[0], mp[1], PERSON[0], PERSON[1], 0.0))
            continue
        for link in m.iter('link'):
            lp = [float(v) for v in (link.findtext('pose') or '0 0 0 0 0 0').split()]
            for c in link.iter('collision'):
                size = c.findtext('geometry/box/size')
                if not size:
                    continue
                sx, sy, sz = (float(v) for v in size.split())
                if sz < 0.3:
                    continue
                yaw = mp[5] + lp[5]
                cx = mp[0] + lp[0] * math.cos(mp[5]) - lp[1] * math.sin(mp[5])
                cy = mp[1] + lp[0] * math.sin(mp[5]) + lp[1] * math.cos(mp[5])
                out.append((cx, cy, sx, sy, yaw))
    out.append((HUSKY[0], HUSKY[1], HUSKY[2], HUSKY[3], 0.0))
    return out, people


def gt_zones(boxes, x0=-6.0, y0=-7.5, nx=240, ny=300):
    """{로봇: bool 격자} 정답 자기 구역 칸 + 격자 원점."""
    xs = x0 + (np.arange(nx) + 0.5) * RES
    ys = y0 + (np.arange(ny) + 0.5) * RES
    X, Y = np.meshgrid(xs, ys)
    occ = np.zeros((ny, nx), bool)
    for cx, cy, sx, sy, yaw in boxes:
        c, s = math.cos(yaw), math.sin(yaw)
        u = (X - cx) * c + (Y - cy) * s
        v = -(X - cx) * s + (Y - cy) * c
        occ |= (np.abs(u) <= sx / 2) & (np.abs(v) <= sy / 2)
    zones = {}
    for ns, (sx0, sy0) in SPAWN.items():
        seen = np.zeros_like(occ)
        r, c = int((sy0 - y0) / RES), int((sx0 - x0) / RES)
        q = deque([(r, c)])
        seen[r, c] = True
        while q:
            r, c = q.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < ny and 0 <= cc < nx and not occ[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    q.append((rr, cc))
        inside = (np.abs(X) < BUILDING[0]) & (np.abs(Y) < BUILDING[1])   # 바깥벽 틈으로 건물 밖이 새어 들어오는 것 막기
        zones[ns] = seen & inside & ((Y < 0) if sy0 < 0 else (Y > 0))
    return zones, (x0, y0, X, Y)


def coverage(npz, zones, grid):
    z = np.load(npz, allow_pickle=True)
    m, (res, ox, oy) = z['map'], z['map_info'][:3]
    _, _, X, Y = grid
    c = ((X - ox) / res).astype(int)
    r = ((Y - oy) / res).astype(int)
    inside = (r >= 0) & (r < m.shape[0]) & (c >= 0) & (c < m.shape[1])
    known = np.zeros(X.shape, bool)
    known[inside] = m[r[inside], c[inside]] >= 0
    return {ns: 100.0 * (known & g).sum() / g.sum() for ns, g in zones.items()}


def events_metrics(path, cut):
    out = {ns: dict(arrive=0, fall=0, home=0, stuck_s=0.0) for ns in SPAWN}
    found = set()
    starts = {}
    with open(path, encoding='utf-8-sig') as f:
        for e in csv.DictReader(f):
            try:
                t = float(e['sim'])
            except (TypeError, ValueError):
                continue
            if t > cut or e['robot'] not in out:
                continue
            o, k, det = out[e['robot']], e['kind'], e.get('detail') or ''
            if k == 'arrive':
                o['arrive'] += 1
            elif k == 'fall':
                o['fall'] += 1
            elif k == 'home' and '도착' in det:
                o['home'] += 1
            elif k == 'stuck_start':
                starts[e['robot']] = t
            elif k == 'stuck_end' and e['robot'] in starts:
                o['stuck_s'] += t - starts.pop(e['robot'])
            elif k == 'survivor' and det.startswith('발견'):
                mm = re.search(r'(person_standing\w*) ([\d.]+) m', det)
                if mm and float(mm.group(2)) <= PERSON_OK:
                    found.add(mm.group(1))
    for ns, t0 in starts.items():
        out[ns]['stuck_s'] += cut - t0
    return out, found


def nearest_progress(run_dir, t):
    p = os.path.join(run_dir, 'progress', f'map_{t:04d}s.npz')
    return p if os.path.exists(p) else None


def main():
    root, world = sys.argv[1], sys.argv[2]
    runs = sys.argv[3:]
    bundle = {}
    with open(os.path.join(root, 'runs.csv'), encoding='utf-8-sig') as f:
        for r in csv.DictReader(f):
            bundle[r['run']] = r['bundle']
            if not sys.argv[3:] and r['run'] not in runs:
                runs.append(r['run'])
    boxes, people = boxes_from_world(world)
    zones, grid = gt_zones(boxes)
    print(f'정답 구역 넓이: ' + ', '.join(f'{ns} {g.sum() * RES * RES:.1f} m²' for ns, g in zones.items()) + f', 마네킹 {len(people)}')
    rows = []
    for run in runs:
        d = os.path.join(root, run)
        ev = os.path.join(d, 'events.csv')
        if not os.path.exists(ev):
            continue
        row = dict(run=run, bundle=bundle.get(run, '?'))
        for cut in CUTS:
            p = nearest_progress(d, cut)
            cov = coverage(p, zones, grid) if p else {}
            for ns in SPAWN:
                row[f'cov{cut}_{ns}'] = round(cov[ns], 1) if ns in cov else ''
        m, found = events_metrics(ev, CUTS[-1])
        for ns in SPAWN:
            for k, v in m[ns].items():
                row[f'{k}_{ns}'] = round(v, 1) if isinstance(v, float) else v
        row['people'] = len(found)
        rows.append(row)
    if not rows:
        return
    keys = list(rows[0])
    with open(os.path.join(root, 'compare.csv'), 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    hdr = ['run', 'bundle', 'cov300 남/북 %', 'cov600 남/북 %', '도착 600', '멈춤 s', '넘어짐', '홈', f'마네킹/{len(people)}']
    print(' | '.join(hdr))
    for r in rows:
        print(' | '.join(str(x) for x in (
            r['run'], r['bundle'], f"{r['cov300_a1_1']}/{r['cov300_a1_2']}", f"{r['cov600_a1_1']}/{r['cov600_a1_2']}",
            r['arrive_a1_1'] + r['arrive_a1_2'], round(r['stuck_s_a1_1'] + r['stuck_s_a1_2']),
            r['fall_a1_1'] + r['fall_a1_2'], r['home_a1_1'] + r['home_a1_2'], r['people'])))


if __name__ == '__main__':
    main()
