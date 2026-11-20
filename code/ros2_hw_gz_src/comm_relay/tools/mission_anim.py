#!/usr/bin/env python3
"""임무 기록 -> 위에서 본 2D 배속 영상 (발표용, 2026-11-20). 시뮬을 다시 돌리지 않고 run_mission.sh 결과 폴더만 쓴다.

  python3 mission_anim.py <결과 폴더> [--speed=20] [--fps=15] [--t0=시뮬초] [--t1=시뮬초] [--out=파일.mp4]
  입력: motion.csv (A1 정답 위치·yaw·상태, 2 Hz), timeline.csv (Husky 위치·단계·사람 수, 10 s), mission.log (사람 발견 보고 위치)
  배경: disaster_sim/scripts/make_world.py 의 배치 (같은 원본). 위 = 전체 지도, 아래 = 로봇 따라 확대.
  배속은 시뮬 시간 기준 (벽시계 아님) 이고 화면에 적는다. 한글 글꼴이 없어 화면 글자는 영어.
"""
import csv
import math
import os
import re
import sys

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                 # noqa: E402
import numpy as np                              # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'disaster_sim', 'scripts'))
import make_world as mw                         # noqa: E402

COLOR = dict(wall='k', box='saddlebrown', ramp='0.8', pillar='peru', debris='olive')
ROBOT = {'a1_1': 'tab:green', 'a1_2': 'tab:purple'}
TRAIL = 60.0        # 지나온 길 표시 (시뮬 s)


def layout():
    """make_world.main() 과 같은 순서로 배치를 채운다 (파일은 안 씀)."""
    for _, cx, north, south in mw.SETS:
        mw.building(cx, 1, north)
        mw.building(cx, -1, south)
    mw.common()
    mw.wall(*mw.START['small_wall'], h=0.6)
    return mw.shapes, mw.people


def draw_static(ax, shapes, people):
    for kind, x, y, sx, sy, yaw in shapes:
        c, s = math.cos(yaw), math.sin(yaw)
        ax.add_patch(Polygon([(x + c * a - s * b, y + s * a + c * b) for a, b in (
            (-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))], color=COLOR[kind])
                     if sy else Circle((x, y), sx, color=COLOR[kind]))
    ax.plot([p[0] for p in people], [p[1] for p in people], '*', color='0.6', ms=9, zorder=2)
    ax.axhspan(-mw.ROAD, mw.ROAD, color='khaki', alpha=0.25, zorder=0)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])


def load(d):
    with open(os.path.join(d, 'motion.csv')) as f:
        rows = list(csv.DictReader(f))
    mo = {'t': np.array([float(r['t']) for r in rows])}
    for ns in ROBOT:
        for k in ('gt_x', 'gt_y', 'gt_yaw'):
            mo[f'{ns}_{k}'] = np.array([float(r[f'{ns}_{k}']) for r in rows])
        mo[f'{ns}_state'] = [r[f'{ns}_state'] for r in rows]
    with open(os.path.join(d, 'timeline.csv')) as f:
        tl = list(csv.DictReader(f))
    reports = []
    with open(os.path.join(d, 'mission.log'), errors='replace') as f:
        for line in f:
            m = re.search(r'사람 발견 보고: (\S+) 월드 \(([-\d.]+), ([-\d.]+)\)', line)
            if m:
                reports.append((float(m.group(2)), float(m.group(3))))
    return mo, tl, reports


def robot_patch(x, y, yaw, color):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(0.45, 0.0), (-0.3, 0.22), (-0.3, -0.22)]        # A1 몸 길이 약 0.6 m, 앞이 뾰족
    return Polygon([(x + c * a - s * b, y + s * a + c * b) for a, b in pts], color=color, zorder=5)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    opt = dict(a[2:].split('=', 1) for a in sys.argv[1:] if a.startswith('--'))
    d = args[0]
    speed, fps = float(opt.get('speed', 20)), float(opt.get('fps', 15))
    mo, tl, reports = load(d)
    t0 = float(opt.get('t0', mo['t'][0]))
    t1 = float(opt.get('t1', mo['t'][-1]))
    out = opt.get('out', os.path.join(d, f'mission_x{speed:g}.mp4'))
    tt = np.array([float(r['t']) for r in tl])
    hx = np.array([float(r['husky_x']) for r in tl])
    hy = np.array([float(r['husky_y']) for r in tl])
    shapes, people = layout()
    n_people = len(people)
    set_names = [s for s, *_ in mw.SETS] + ['common']

    fig = plt.figure(figsize=(19.2, 10.8), dpi=100)
    ax_all = fig.add_axes([0.01, 0.70, 0.98, 0.22])
    ax = fig.add_axes([0.01, 0.01, 0.98, 0.67])
    for a in (ax_all, ax):
        draw_static(a, shapes, people)
    ax_all.set_xlim(-5, 68)
    ax_all.set_ylim(-8, 8)
    half_w, half_h = 17.0, 7.5          # 확대 창 (m): 34 x 15 -> 아래 칸 비율과 맞춤
    ax.set_ylim(-half_h, half_h)
    title = fig.text(0.01, 0.955, '', fontsize=20, family='monospace', va='center')
    fig.text(0.99, 0.955, f'x{speed:g} speed (simulation time)', fontsize=18, ha='right', va='center', color='firebrick')
    handles = [plt.Line2D([], [], marker='^', ls='', color=c, ms=12, label=f'{ns}  (A1)') for ns, c in ROBOT.items()]
    handles += [plt.Line2D([], [], marker='s', ls='', color='gold', mec='k', ms=12, label='husky'),
                plt.Line2D([], [], marker='*', ls='', color='0.6', ms=14, label='person (truth)'),
                plt.Line2D([], [], marker='*', ls='', color='red', ms=14, label='person found')]
    ax.legend(handles=handles, loc='upper right', fontsize=13, framealpha=0.9)

    writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), fps, (1920, 1080))
    dyn, cx = [], None
    n_frames = int((t1 - t0) * fps / speed) + 1
    for i in range(n_frames):
        t = t0 + i * speed / fps
        for art in dyn:
            art.remove()
        dyn = []
        k = min(int(np.searchsorted(mo['t'], t)), len(mo['t']) - 1)
        j = max(int(np.searchsorted(tt, t, side='right')) - 1, 0)
        row = tl[j]
        hxi, hyi = float(np.interp(t, tt, hx)), float(np.interp(t, tt, hy))
        found = reports[:int(row['people'])]
        states = []
        for a in (ax_all, ax):
            for ns, col in ROBOT.items():
                lo = int(np.searchsorted(mo['t'], t - TRAIL))
                dyn += a.plot(mo[f'{ns}_gt_x'][lo:k + 1], mo[f'{ns}_gt_y'][lo:k + 1], '-', color=col, lw=1.5, alpha=0.6, zorder=4)
                dyn.append(a.add_patch(robot_patch(mo[f'{ns}_gt_x'][k], mo[f'{ns}_gt_y'][k], mo[f'{ns}_gt_yaw'][k], col)))
            dyn += a.plot(hxi, hyi, 's', color='gold', mec='k', ms=10 if a is ax_all else 16, zorder=5)
            if found:
                dyn += a.plot([p[0] for p in found], [p[1] for p in found], '*', color='red', ms=10 if a is ax_all else 20, zorder=6)
        for ns in ROBOT:
            states.append(f'{ns} {mo[f"{ns}_state"][k]:<9}')
        target = (mo['a1_1_gt_x'][k] + mo['a1_2_gt_x'][k] + hxi) / 3      # 확대 창은 세 로봇 가운데를 부드럽게 따라감
        cx = target if cx is None else cx + 0.08 * (target - cx)
        ax.set_xlim(cx - half_w, cx + half_w)
        dyn.append(ax_all.add_patch(Rectangle((cx - half_w, -half_h), 2 * half_w, 2 * half_h, fill=False, ec='royalblue', lw=1.5)))
        si = int(row['set'])
        title.set_text(f't = {t:6.0f} s   {row["phase"]:<8} {set_names[si] if si < len(set_names) else "":<7}'
                       f' people {len(found)}/{n_people}    ' + '  '.join(states))
        fig.canvas.draw()
        img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        if i % 100 == 0:
            print(f'{i}/{n_frames} 장면', flush=True)
    writer.release()
    print(f'영상: {out} ({n_frames} 장면, {n_frames / fps:.0f} s, 시뮬 {t0:.0f}~{t1:.0f} s)')


if __name__ == '__main__':
    main()
