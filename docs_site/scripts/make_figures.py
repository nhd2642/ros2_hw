"""문서에 쓰는 결과 그래프를 그린다 (기본은 저장소에 들어 있는 PASS 실행 2 개).

    python3 docs_site/scripts/make_figures.py                       저장소의 PASS 실행 2 개 (기본)
    python3 docs_site/scripts/make_figures.py <실행> [<실행> ...]     첫 실행 = 경로·SLAM·사람 그래프, 전부 = 단계 시간 그래프
    make -C docs_site figures RUNS="<실행> <실행>"                     위와 같음, make 로 (<실행> = results/ 아래 폴더 이름)

입력: results/<실행>/timeline.csv (mission_watch.py 가 시뮬 10 초마다 쓴 표),
      code/ros2_hw_gz_src/disaster_sim/config/mission.yaml (구역, 주차점, 사람 정답).
출력: docs_site/images/figures/*.png. 두 언어가 같은 그림을 쓰도록 라벨은 영어.
"""
import csv
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
import yaml  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
OUT = os.path.join(ROOT, 'docs_site', 'images', 'figures')
RUNS = {'20261114_062151_set0': 'run B (comm 0/1 in set 1)', '20261113_035152_set0': 'run A (no comm test)'}
MAIN = '20261114_062151_set0'

# 흑청바지 + 빨간 니트; 로봇마다 색은 모든 그림에서 고정
INK, INK_SOFT, RULE, PAPER = '#1C1F24', '#4A515C', '#E3E2DE', '#FFFFFF'
COLOR = {'a1_1': '#C8102E', 'a1_2': '#4C6A92', 'husky': '#2A3038'}
STYLE = {'a1_1': '-', 'a1_2': '-', 'husky': '--'}
PHASE_FILL = {'TRANSIT': '#ECEBE7', 'EXPLORE': '#FFFFFF', 'REGROUP': '#F6DADF'}
SET_NAMES = ['set 1', 'set 2', 'set 3', 'common']

plt.rcParams.update({
    'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.edgecolor': RULE, 'axes.labelcolor': INK_SOFT,
    'xtick.color': INK_SOFT, 'ytick.color': INK_SOFT, 'axes.titlecolor': INK, 'axes.titleweight': 'bold',
    'axes.titlesize': 12, 'axes.grid': True, 'grid.color': RULE, 'grid.linewidth': 0.6, 'axes.spines.top': False,
    'axes.spines.right': False, 'figure.facecolor': PAPER, 'axes.facecolor': PAPER, 'legend.frameon': False,
    'savefig.dpi': 160, 'savefig.bbox': 'tight',
})


def load(run):
    with open(os.path.join(ROOT, 'results', run, 'timeline.csv')) as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k, v in r.items():
            try:
                r[k] = float(v)
            except ValueError:
                pass
    return rows


def phases(rows):
    """연속한 표본에서 [(단계, 세트, 시작, 끝)] 구간을 만든다."""
    spans, cur = [], None
    for r in rows:
        key = (r['phase'], int(r['set']))
        if cur is None or key != cur[:2]:
            if cur:
                spans.append((*cur[:2], cur[2], r['t']))
            cur = (*key, r['t'])
    spans.append((*cur[:2], cur[2], rows[-1]['t']))
    return spans


def shade(ax, rows):
    for ph, k, t0, t1 in phases(rows):
        ax.axvspan(t0, t1, color=PHASE_FILL.get(ph, PAPER), zorder=0, lw=0)
        if ph == 'EXPLORE' and k < len(SET_NAMES):
            ax.text((t0 + t1) / 2, 1.0, SET_NAMES[k], transform=ax.get_xaxis_transform(), ha='center', va='bottom',
                    fontsize=9, color=INK_SOFT)
    off = [r['t'] for r in rows if r['comm'] == 0]
    if off:
        ax.axvspan(min(off), max(off), color='#C8102E', alpha=0.14, zorder=0, lw=0)
        ax.text((min(off) + max(off)) / 2, 0.97, 'comm 0', transform=ax.get_xaxis_transform(), ha='center',
                va='top', fontsize=8, color='#8F0B20')


def trajectories(rows, cfg):
    fig, ax = plt.subplots(figsize=(12, 4.2))
    for s in cfg['sets']:
        for ns, (x0, x1, y0, y1) in s['zones'].items():
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=True, fc='#F4F3F0', ec='#B9BDC4', lw=0.8, zorder=1))
        ax.plot(*s['park'], marker='s', ms=8, color=COLOR['husky'], zorder=4, mfc='white', mew=1.6)
        ax.annotate(s['name'], s['park'], xytext=(0, -16), textcoords='offset points', ha='center', fontsize=8,
                    color=INK_SOFT)
    ax.axhspan(-1.5, 1.5, color='#ECEBE7', zorder=0)   # 도로 (x 축 따라 폭 3 m)
    for ns in ('husky', 'a1_1', 'a1_2'):
        ax.plot([r[f'{ns}_x'] for r in rows], [r[f'{ns}_y'] for r in rows], STYLE[ns], color=COLOR[ns], lw=1.8,
                label=ns, zorder=3)
    px, py = zip(*cfg['people_truth'])
    ax.scatter(px, py, marker='*', s=120, color='#E2283F', edgecolor='white', linewidth=0.8, zorder=5,
               label='person (truth)')
    ax.set_aspect('equal')
    ax.set_xlabel('world x [m]')
    ax.set_ylabel('world y [m]')
    ax.set_title(f'Robot paths, full mission ({MAIN}, 10 s samples)', loc='left')
    ax.legend(loc='upper left', bbox_to_anchor=(1.0, 1.0))
    fig.savefig(os.path.join(OUT, 'trajectories.png'))
    plt.close(fig)


def slam_error(rows):
    fig, ax = plt.subplots(figsize=(12, 3.4))
    shade(ax, rows)
    t = [r['t'] for r in rows]
    for ns in ('a1_1', 'a1_2'):
        ax.plot(t, [r[f'{ns}_slam_err'] for r in rows], color=COLOR[ns], lw=2, label=ns)
    ax.axhline(1.0, color=INK_SOFT, lw=0.8, ls=':')
    ax.text(t[-1], 1.0, ' 1 m', va='center', fontsize=8, color=INK_SOFT)
    ax.set_xlim(0, t[-1])
    ax.set_ylim(0, None)
    ax.set_xlabel('mission sim time [s]   (grey = TRANSIT, white = EXPLORE)')
    ax.set_ylabel('shared pose error [m]')
    ax.set_title('SLAM shared-pose error vs. Gazebo ground truth', loc='left', pad=16)
    ax.legend(loc='upper left', bbox_to_anchor=(1.0, 1.0))
    fig.savefig(os.path.join(OUT, 'slam_error.png'))
    plt.close(fig)


def people(rows):
    fig, ax = plt.subplots(figsize=(12, 2.6))
    shade(ax, rows)
    t = [r['t'] for r in rows]
    ax.step(t, [r['people'] for r in rows], where='post', color=COLOR['a1_1'], lw=2)
    ax.set_xlim(0, t[-1])
    ax.set_ylim(0, 9.8)
    ax.set_yticks(range(0, 10, 3))
    ax.set_xlabel('mission sim time [s]')
    ax.set_ylabel('people reported')
    ax.set_title('Survivor reports received by Husky (9 placed)', loc='left', pad=16)
    fig.savefig(os.path.join(OUT, 'people.png'))
    plt.close(fig)


def summary_phases(run):
    """summary.txt 의 '=== 요약' 블록에서 정확한 구간을 읽는다: '  EXPLORE  세트 0:     57 ->    322 s (  265 s)'."""
    spans = []
    for line in open(os.path.join(ROOT, 'results', run, 'summary.txt'), encoding='utf-8'):
        m = re.match(r'\s+([A-Z]+)\s+\S+\s+(\d+):\s+(\d+) ->\s+(\d+) s \(\s*(\d+) s\)', line)
        if m:
            spans.append((m[1], int(m[2]), float(m[3]), float(m[3]) + float(m[5])))   # 괄호 안 걸린 시간이 정확한 값
    return spans


def phase_bars():
    fig, ax = plt.subplots(figsize=(12, 2.4))
    ax.set_axisbelow(True)
    for i, (run, label) in enumerate(RUNS.items()):
        for ph, k, t0, t1 in summary_phases(run):
            if ph == 'DONE':
                continue
            c = COLOR['a1_1'] if ph == 'EXPLORE' else '#B9BDC4'
            ax.barh(i, t1 - t0, left=t0, color=c, edgecolor='white', linewidth=2, height=0.55)
            if ph == 'EXPLORE' and k < len(SET_NAMES):
                ax.text((t0 + t1) / 2, i, f'{SET_NAMES[k]}\n{t1 - t0:.0f} s', ha='center', va='center', color='white',
                        fontsize=8, fontweight='bold')
    ax.set_yticks(range(len(RUNS)), list(RUNS.values()))
    ax.invert_yaxis()
    ax.set_xlim(0, None)
    ax.set_xlabel('mission sim time [s]   (red = EXPLORE, grey = TRANSIT)')
    ax.set_title('Phase timeline of the two committed PASS runs', loc='left')
    ax.grid(axis='y', visible=False)
    fig.savefig(os.path.join(OUT, 'phase_timeline.png'))
    plt.close(fig)


def gait():
    # docs/mission_design.md 5 절 (tools/gait_sweep.sh, 평지, 명령마다 8 s, 정답 위치 기준)
    v = [0.1, 0.2, 0.3, 0.4, 0.5]
    ratio = [99, 95, 107, 108, 107]
    tilt = [2.3, 2.3, 3.7, 4.6, 5.5]
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 3.2))
    for ax, y, lab, title in ((a, ratio, 'actual / command [%]', 'Speed tracking'),
                              (b, tilt, 'max body tilt [deg]', 'Body tilt while walking')):
        ax.bar([str(x) for x in v], y, color=COLOR['a1_1'], width=0.55, edgecolor='white', linewidth=2)
        for i, val in enumerate(y):
            ax.text(i, val, f'{val:g}', ha='center', va='bottom', fontsize=9, color=INK)
        ax.set_xlabel('commanded vx [m/s]')
        ax.set_ylabel(lab)
        ax.set_title(title, loc='left')
        ax.grid(axis='x', visible=False)
    a.axhline(100, color=INK_SOFT, lw=0.8, ls=':')
    a.set_ylim(0, 125)
    fig.savefig(os.path.join(OUT, 'gait_sweep.png'))
    plt.close(fig)


def main():
    global MAIN, RUNS
    if len(sys.argv) > 1:      # 새 실행: 폴더 이름을 라벨로
        RUNS = {r: r for r in sys.argv[1:]}
        MAIN = sys.argv[1]
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(ROOT, 'code', 'ros2_hw_gz_src', 'disaster_sim', 'config', 'mission.yaml')) as f:
        cfg = yaml.safe_load(f)
    rows = load(MAIN)
    trajectories(rows, cfg)
    slam_error(rows)
    people(rows)
    phase_bars()
    gait()
    print('figures ->', OUT)


if __name__ == '__main__':
    main()
