#!/usr/bin/env python3
"""mission.world + mission.yaml 생성기 (배치의 유일한 원본).

  python3 make_world.py            # ../worlds/mission.world, ../config/mission.yaml 다시 씀
  python3 make_world.py layout.png # + 위에서 본 배치 그림 (matplotlib 필요)

배치 (월드 좌표 m, 도로 = x 축, 폭 3 m):
  출발(Husky 0,0)  세트1 x 4~12  세트2 x 16~24  세트3 x 28~36  세트4(어려움) x 40~48  공동 건물 x 52~66, y -6~6
  세트 = 도로 양옆 8x5 m 건물 한 쌍 (남 = a1_1, 북 = a1_2), 문은 도로 쪽 가운데 1.6 m.
건물 안 좌표 (u, v): u = 도로 방향(문 가운데 0, -4~4), v = 문에서 안쪽(0~5). 남쪽 건물은 y 로 뒤집힌다.
라이다 평면이 0.30 m 라 장애물은 0.5 m 이상으로만 만든다 (낮으면 2D 라이다가 못 봄). 세트4 의 낮은 잔해(debris)만 예외.
세트4 (2026-11-19) 는 아래 배치 규칙을 일부러 어기는 '규칙 밖 시험 장면'이다: 보행·Nav·SLAM 의 강건성을 재려고 좁은 길(1.2 m, 90° 꺾임 2 번),
  1.0 m 문, 물체 밀집 방(틈 1.0~1.2 m), 라이다에 안 보이는 낮은 잔해(0.12 m, 깊이 카메라 지형 층만 봄)를 둔다.
  공동 건물은 안의 배치 그대로 12 m 동쪽으로 옮겼다 (COMMON_DX).
"""
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
T, H = 0.15, 2.0            # 벽 두께, 높이
ROAD, W, D, DOOR = 1.5, 4.0, 5.0, 0.8   # 도로 반폭, 건물 반너비, 깊이, 문 반폭
SLOT = {'a1_1': (-1.6, -0.8), 'a1_2': (-1.6, 0.8)}   # Husky 기준 추종 자리 (뒤 1.6 m, 좌우 0.8 m)

# 건물 안 배치: walls (u1, v1, u2, v2), boxes (u, v, 가로, 세로), pillars (u, v), ramps (꼭대기 u, v, 진행 방향 'u'|'v', 반폭), people (u, v)
# 배치 규칙 (2026-11-12 세트 1 시험 뒤). 비용지도에서는 벽·난간이 양쪽 0.3 m 씩 두꺼워져 폭 1.7 m 통로가 1.0 m 로 줄고,
# 그 폭에서는 A1 추종(DWB)이 제자리에서 돌다 실패했다:
#   - 지나다니는 통로·방은 폭 2.0 m 이상. 1.6 m 는 곧장 지나는 문·개구부에만.
#   - 지나자마자(또는 지나기 직전에) 90 도 넘게 꺾어야 하는 개구부(벽 끝을 돌아 들어가는 방 입구)는 2.4 m 이상.
#     1.6 m 였을 때 세트 1 북쪽 뒷방 문, 세트 1 남쪽 안쪽 방 입구, 세트 2 북쪽 왼쪽 방 입구에서 추종 실패·갇힘이 되풀이됐다.
#   - 장애물·사람은 벽에 붙이거나(벽 중심에서 0.4 m) 둘레 1.3 m 이상 비움 (사람과 뒷벽 사이 0.9 m 틈에 들어가 끼었음).
#   - 경사로는 벽에서 벽까지 방 폭 전체를 덮는 오르내림 둔덕 (옆 가장자리·난간 없음, 8 도). 난간 달린 폭 1.9 m 경사로는
#     난간 끝·옆에 끼어 제자리걸음 -> 다리 odom 만 앞으로 가서 SLAM 위치가 2 m 넘게 흘렀다 (원본 맵의 넘어짐 4 번도 전부 경사로 옆 가장자리).
INTERIORS = {
    'n1': dict(walls=[(-4, 2.6, -1.4, 2.6), (0.2, 2.6, 4, 2.6)],
               boxes=[(-3.6, 1.3, 0.6, 0.6)], people=[(2.6, 4.6)]),
    's1': dict(walls=[(0.9, 1.8, 0.9, 5)],
               boxes=[(-3.6, 4.6, 0.6, 0.6)], ramps=[(-1.55, 2.9, 'v', 2.37)], people=[(2.5, 4.6)]),
    'n2': dict(walls=[(-1.2, 0, -1.2, 2.6), (1.2, 2.4, 1.2, 5)],
               boxes=[(3.6, 4.6, 0.6, 0.6)], people=[(-3.6, 1.4)]),
    's2': dict(walls=[(-4, 3.0, -1.5, 3.0)],
               pillars=[(-2, 1.6), (2, 1.6), (2, 3.6)], boxes=[(0.2, 2.6, 0.8, 0.5), (3.6, 4.6, 0.6, 0.6)],
               people=[(-3.0, 4.6)]),
    'n3': dict(walls=[(0.5, 3.0, 0.5, 5)],
               ramps=[(0.0, 2.0, 'v', 3.92)], boxes=[(3.6, 4.6, 0.6, 0.6)], people=[(-2.8, 4.6)]),
    's3': dict(walls=[(-4, 2.8, 1.2, 2.8)],
               boxes=[(-3.6, 1.4, 0.6, 0.6)], pillars=[(2.6, 1.4)], people=[(-2.6, 4.6)]),
    # 세트4 = 규칙 밖 시험 장면 (머리말). n4: 물체 밀집 방 — 기둥 줄(틈 1.0 m, 문 바로 앞) + 상자 줄(틈 1.1 m) + 낮은 잔해 2 개, 사람은 오른쪽 뒤 구석.
    'n4': dict(pillars=[(-2.2, 1.9), (-0.8, 1.9), (0.6, 1.9)],
               boxes=[(-1.6, 3.4, 0.5, 0.5), (0.0, 3.4, 0.5, 0.5), (1.6, 3.4, 0.5, 0.5)],
               debris=[(2.6, 1.6, 0.4, 0.4), (-3.0, 3.0, 0.4, 0.4)], people=[(3.6, 4.6)]),
    # s4: 좁은 길 — 문에서 안쪽으로 폭 1.2 m 복도(벽 중심 u ±0.675) -> 뒷벽 앞에서 오른쪽으로 꺾는 1.2 m 복도(v 3.65~5)
    #     -> 다시 꺾어 1.2 m 틈으로 안쪽 방(사람). 복도 왼쪽 벽에 1.0 m 문(왼쪽 방, 상자).
    's4': dict(walls=[(-0.675, 0, -0.675, 1.0), (-0.675, 2.15, -0.675, 5),        # 왼쪽 벽, v 1.0~2.15 = 1.0 m 문
                      (0.675, 0, 0.675, 3.65), (0.675, 3.65, 2.65, 3.65)],         # 오른쪽 벽 + 꺾인 복도 바닥 벽 (끝 ~ 옆벽 1.2 m 틈)
               boxes=[(-2.6, 3.6, 0.6, 0.6)], people=[(2.6, 0.4)]),
}
COMMON_DX = 12.0     # 공동 건물을 동쪽으로 옮긴 거리 (세트4 자리)
SETS = [('set1', 8.0, 'n1', 's1'), ('set2', 20.0, 'n2', 's2'), ('set3', 32.0, 'n3', 's3'), ('set4', 44.0, 'n4', 's4')]
COMMON = dict(x0=40.0, x1=54.0, y0=-6.0, y1=6.0, doors=(-3.0, 3.0), park=(38.0, 0.0),
              walls=[(45, -2, 45, 2), (45, 2, 49, 2), (45, -2, 49, -2), (49, -2, 49, -0.8), (49, 0.8, 49, 2),
                     (47, 3.6, 47, 6), (50.6, -6, 50.6, -4.2), (50.6, -2.6, 54, -2.6)],
              boxes=[(42.5, 0.0, 0.8, 0.6), (51.5, 1.5, 0.6, 0.8), (43.5, 4.6, 0.6, 0.6)],
              pillars=[(51.5, -1.2), (42.5, -4.6)], ramps=[(47.0, -4.0, 'x', 1.92)],
              people=[(47.6, 0.0), (51.5, 4.5), (52.6, -4.6)])
START = dict(husky=(0.0, 0.0, 0.0), a1_1=(-2.4, -0.8, 0.0), a1_2=(-2.4, 0.8, 0.0),
             small_wall=(-1.2, -1.0, -1.2, 0.0))        # a1_1 앞 작은 벽 (추종 중 회피 확인용)

out, people, names, shapes = [], [], {}, []      # shapes: 그림용 (종류, x, y, 가로, 세로, yaw) — 원기둥은 세로 0


def name(kind):
    names[kind] = names.get(kind, 0) + 1
    return f'{kind}_{names[kind]}'


def box(kind, x, y, z, sx, sy, sz, yaw=0.0, roll=0.0, pitch=0.0, rgb='0.75 0.75 0.72'):
    g = f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>'
    shapes.append((kind, x, y, sx, sy, yaw))
    out.append(f'<model name="{name(kind)}"><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>'
               f'<link name="link"><collision name="c">{g}</collision><visual name="v">{g}'
               f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual></link></model>')


def wall(x1, y1, x2, y2, h=H):
    L = math.hypot(x2 - x1, y2 - y1)
    box('wall', (x1 + x2) / 2, (y1 + y2) / 2, h / 2, L + T, T, h, yaw=math.atan2(y2 - y1, x2 - x1))


def pillar(x, y):
    g = '<geometry><cylinder><radius>0.2</radius><length>1.2</length></cylinder></geometry>'
    shapes.append(('pillar', x, y, 0.2, 0, 0))
    out.append(f'<model name="{name("pillar")}"><static>true</static><pose>{x:.3f} {y:.3f} 0.6 0 0 0</pose><link name="link">'
               f'<collision name="c">{g}</collision><visual name="v">{g}<material><ambient>0.5 0.4 0.3 1</ambient>'
               f'<diffuse>0.5 0.4 0.3 1</diffuse></material></visual></link></model>')


def ramp(x, y, along, half, L=1.0, deg=8.0, t=0.04):
    """(x, y) 가 꼭대기인 오르내림 둔덕 (판 2 장, 진행 방향 along = 'x' | 'y', 폭 2*half 는 양옆 벽에 닿게 준다)."""
    a = math.radians(deg)
    d, z = L / 2 * math.cos(a), L / 2 * math.sin(a) - t / 2 * math.cos(a)
    for s in (-1, 1):
        if along == 'x':
            box('ramp', x + s * d, y, z, L, 2 * half, t, pitch=s * a, rgb='0.45 0.45 0.45')
        else:
            box('ramp', x, y + s * d, z, 2 * half, L, t, roll=-s * a, rgb='0.45 0.45 0.45')


def person(x, y, yaw):
    people.append([round(x, 2), round(y, 2)])
    out.append(f'<include><name>person_{len(people)}</name><static>true</static><pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.3f}</pose>'
               f'<uri>model://person_standing</uri></include>')


def building(cx, side, key):
    """세트 건물 한 채. side = +1 북, -1 남. 돌려줌: 구역 사각형 [x0, x1, y0, y1]."""
    it = INTERIORS[key]
    P = lambda u, v: (cx + u, side * (ROAD + v))
    for u1, v1, u2, v2 in [(-W, 0, -DOOR, 0), (DOOR, 0, W, 0), (-W, D, W, D), (-W, 0, -W, D), (W, 0, W, D)] + it.get('walls', []):
        wall(*P(u1, v1), *P(u2, v2))
    for u, v, su, sv in it.get('boxes', []):
        box('box', *P(u, v), 0.35, su, sv, 0.7, rgb='0.55 0.35 0.2')
    for u, v, su, sv in it.get('debris', []):              # 낮은 잔해 0.12 m: 라이다 평면 아래, 깊이 카메라만 봄
        box('debris', *P(u, v), 0.06, su, sv, 0.12, rgb='0.35 0.3 0.25')
    for u, v in it.get('pillars', []):
        pillar(*P(u, v))
    for u, v, along, half in it.get('ramps', []):
        ramp(*P(u, v), 'x' if along == 'u' else 'y', half)
    for u, v in it.get('people', []):
        person(*P(u, v), 0.0 if side > 0 else math.pi)     # 문 쪽을 보게
    ys = sorted((side * ROAD, side * (ROAD + D)))
    return [cx - W, cx + W, ys[0], ys[1]]


def common(dx=COMMON_DX):
    c = COMMON
    x0, x1, y0, y1 = c['x0'] + dx, c['x1'] + dx, c['y0'], c['y1']
    edges = sorted([y0] + [d + s * DOOR for d in c['doors'] for s in (-1, 1)] + [y1])
    for a, b in zip(edges[::2], edges[1::2]):               # 서쪽 벽 (문 2 개)
        wall(x0, a, x0, b)
    for seg in [(x0, y0, x1, y0), (x0, y1, x1, y1), (x1, y0, x1, y1)] + \
            [(xa + dx, ya, xb + dx, yb) for xa, ya, xb, yb in c['walls']]:
        wall(*seg)
    for x, y, sx, sy in c['boxes']:
        box('box', x + dx, y, 0.35, sx, sy, 0.7, rgb='0.55 0.35 0.2')
    for x, y in c['pillars']:
        pillar(x + dx, y)
    for x, y, along, half in c['ramps']:
        ramp(x + dx, y, along, half)
    for x, y in c['people']:
        person(x + dx, y, math.pi / 2)
    return [x0, x1, y0, y1]


def main():
    sets = []
    for label, cx, north, south in SETS:
        n0 = len(people)
        zones = {'a1_2': building(cx, 1, north), 'a1_1': building(cx, -1, south)}
        sets.append(dict(name=label, park=[cx - SLOT['a1_1'][0], 0.0], zones=zones, people=len(people) - n0))
    n0 = len(people)
    z = common()
    sets.append(dict(name='common', park=[COMMON['park'][0] + COMMON_DX, COMMON['park'][1]], zones={'a1_1': z, 'a1_2': list(z)}, people=len(people) - n0))
    wall(*START['small_wall'], h=0.6)

    world = f'''<?xml version="1.0"?>
<sdf version="1.7">
  <world name="default">
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"/>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <light name="sun" type="directional">
      <cast_shadows>0</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface></collision>
      <visual name="visual"><cast_shadows>0</cast_shadows><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
    </link></model>
    <gravity>0 0 -9.8</gravity>
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor><real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene><ambient>0.4 0.4 0.4 1</ambient><background>0.7 0.7 0.7 1</background><shadows>0</shadows></scene>
    {(chr(10) + '    ').join(out)}
  </world>
</sdf>
'''
    cfg = dict(spawn={k: list(START[k]) for k in ('husky', 'a1_1', 'a1_2')},
               slots={k: list(v) for k, v in SLOT.items()}, sets=sets, people_truth=people)
    with open(os.path.join(HERE, '..', 'worlds', 'mission.world'), 'w') as f:
        f.write(world)
    with open(os.path.join(HERE, '..', 'config', 'mission.yaml'), 'w') as f:
        f.write('# make_world.py 가 만든 파일 (직접 고치지 말 것). 좌표 = Gazebo 월드 m.\n')
        yaml.safe_dump(cfg, f, sort_keys=False, default_flow_style=None)
    print(f'models {len(out)}, people {len(people)}, sets {[(s["name"], s["people"]) for s in sets]}')
    if len(sys.argv) > 1:
        draw(cfg, sys.argv[1])


def draw(cfg, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Rectangle
    color = dict(wall='k', box='saddlebrown', ramp='0.75', pillar='peru', debris='olive')
    fig, ax = plt.subplots(figsize=(22, 6.5))
    for kind, x, y, sx, sy, yaw in shapes:
        c, s = math.cos(yaw), math.sin(yaw)
        ax.add_patch(Polygon([(x + c * a - s * b, y + s * a + c * b) for a, b in (
            (-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))], color=color[kind])
                     if sy else Circle((x, y), sx, color=color[kind]))
    for x, y in cfg['people_truth']:
        ax.plot(x, y, 'r*', ms=12)
    for st in cfg['sets']:
        ax.plot(*st['park'], 'bs', ms=9)
        ax.text(st['park'][0], 0.5, st['name'], color='b', ha='center')
        for ns, (x0, x1, y0, y1) in st['zones'].items():
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec='g' if ns == 'a1_1' else 'm', ls='--', lw=0.8))
    for k, (x, y, _) in cfg['spawn'].items():
        ax.plot(x, y, 'o', color='gold' if k == 'husky' else 'c', ms=10)
        ax.text(x, y - 0.6, k, ha='center', fontsize=8)
    ax.set_aspect('equal')
    ax.set_xlim(-5, 68)
    ax.set_ylim(-8, 8)
    ax.grid(alpha=0.3)
    ax.set_title('mission.world  (red star = person, blue square = Husky parking, gray = ramp hump, dashed = zones)')
    fig.tight_layout()
    fig.savefig(path, dpi=80)


if __name__ == '__main__':
    main()
