# 6. 월드 배치 바꾸기

건물 안 벽·상자·경사로·사람 위치를 바꾸고 싶을 때는 **`make_world.py` 하나만** 고칩니다. 이 스크립트가 월드 파일과 임무 설정을 함께 다시 만들기 때문에, 노드 코드에는 좌표가 들어 있지 않습니다.

```text
disaster_sim/scripts/make_world.py   ──►  disaster_sim/worlds/mission.world   (Gazebo 월드)
          (배치의 유일한 원본)       └──►  disaster_sim/config/mission.yaml   (스폰, 추종 자리, 세트별 주차점·구역·사람 수, 사람 정답)
```

:::{warning}
`mission.world` 와 `mission.yaml` 을 **직접 고치지 마세요.** 다음에 `make_world.py` 를 실행하면 덮어씌워집니다.
:::

## 좌표 약속

- 월드 좌표(m), **도로 = x 축**(폭 3 m). Husky 는 (0, 0) 에서 출발.
- 세트 1 = x 4~12, 세트 2 = x 16~24, 세트 3 = x 28~36, 세트 4 = x 40~48, 공동 건물 = x 52~66, y −6~6.
- 세트마다 도로 양옆에 8 × 5 m 건물 한 쌍(남 = `a1_1`, 북 = `a1_2`), 문은 도로 쪽 가운데 1.6 m.
- 건물 안 배치는 **건물 좌표 (u, v)** 로 적습니다: u = 도로 방향(문 가운데 0, −4~4), v = 문에서 안쪽(0~5). 남쪽 건물은 y 로 뒤집힙니다.

## 배치 적는 법

`INTERIORS` 사전에 건물 이름(`n1`=세트 1 북쪽, `s1`=세트 1 남쪽 …)마다 적습니다.

```python
INTERIORS = {
    'n1': dict(walls=[(-4, 2.6, -1.4, 2.6), (0.2, 2.6, 4, 2.6)],   # (u1, v1, u2, v2) 벽 선분
               boxes=[(-3.6, 1.3, 0.6, 0.6)],                       # (u, v, 가로, 세로) 상자
               people=[(2.6, 4.6)]),                                # (u, v) 사람
    's1': dict(walls=[(0.9, 1.8, 0.9, 5)],
               boxes=[(-3.6, 4.6, 0.6, 0.6)],
               ramps=[(-1.55, 2.9, 'v', 2.37)],                     # (꼭대기 u, v, 진행 방향 'u'|'v', 반폭)
               people=[(2.5, 4.6)]),
    ...
}
```

`pillars=[(u, v)]` 로 기둥도 넣을 수 있습니다.

## 꼭 지킬 배치 규칙

시험하며 정한 규칙입니다. 어기면 A1 이 끼이거나 추종에 실패합니다 (이유는 {doc}`../concepts/world_layout`).

:::{important}
1. 지나다니는 통로·방은 **폭 2.0 m 이상**. 1.6 m 는 곧장 지나는 문·개구부에만.
2. 지나자마자 90° 넘게 꺾어야 하는 개구부(벽 끝을 돌아 들어가는 방 입구)는 **2.4 m 이상**.
3. 장애물·사람은 **벽에 붙이거나**(벽 중심에서 0.4 m) **둘레 1.3 m 이상 비움**.
4. 경사로는 **벽에서 벽까지 방 폭 전체**를 덮는 오르내림 둔덕(8°, 옆 가장자리·난간 없음).
5. 장애물은 **높이 0.5 m 이상**만 (라이다 평면이 0.30 m 라 그보다 낮으면 2D 라이다가 못 봄).
:::

## 다시 만들기

```bash
cd ~/ros2_hw/ws/src/disaster_sim/scripts
python3 make_world.py                 # mission.world, mission.yaml 다시 씀
python3 make_world.py layout.png      # + 위에서 본 배치 그림
```

`layout.png` 는 문서의 배치도(`docs/img/mission_layout.png`)와 같은 형식입니다. 빨간 별 = 사람, 파란 네모 = Husky 주차점, 회색 = 경사로, 점선 = 구역.

![배치도](../../images/mission_layout.png)

`--symlink-install` 로 빌드했다면 다시 빌드할 필요 없이 바로 시뮬을 띄울 수 있습니다.

```bash
SETS=1 bash ~/ros2_hw/ws/src/comm_relay/tools/run_mission.sh 0     # 바꾼 세트 1 만 확인
```

## 원본 맵 띄워 보기

이전 구조에서 쓰던 건물 한 채짜리 맵은 월드만 띄울 수 있습니다 (지금 임무 구조로는 돌려 보지 않았습니다).

```bash
ros2 launch disaster_sim world.launch.py world:=disaster.world
```
