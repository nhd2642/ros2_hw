# 지형 인식

2D 라이다는 한 평면(높이 약 0.30 m)만 봅니다. 그래서 낮은 턱, 경사로 옆 가장자리, 낭떠러지는 못 봅니다. `terrain_perception` 패키지의 `terrain_node` 는 앞쪽을 보는 **깊이 센서 점구름**으로 이런 위험을 찾아 Nav2 비용지도에 넣습니다.

## 입력과 출력

| 방향 | 토픽 | 내용 |
| --- | --- | --- |
| 입력 | `/<ns>/terrain_camera/points` | 깊이 점구름 (160 × 120, 수평 ±43°, 수직 ±35°, 0.1~4 m, 5 Hz) |
| 입력 | `/<ns>/odom` | 로봇 위치 (odom 프레임) |
| 입력 | `/<ns>/imu/data` | 몸통 roll/pitch |
| 입력 | `/<ns>/scan_ready` | 라이다가 맞힌 칸 (억제용) |
| 입력 | `/mission/state`, `/husky/pose`, `/<상대>/pose_shared` | 단계, 다른 로봇 위치 |
| 출력 | `/<ns>/terrain_hazards` | 위험 칸 중심 → 비용지도 marking |
| 출력 | `/<ns>/terrain_free` | 로봇 둘레 원 위 점 → 비용지도 clearing |

## 위험 판정 (grid.py)

점구름을 IMU roll/pitch 로 수평으로 돌린 뒤 0.05 m 칸에 높이를 모읍니다. 다음 중 하나면 위험 칸입니다 (`max_step` = 0.04 m = CHAMP `swing_height`).

1. 칸 안 높이 폭 > `max_step` — 수직면, 턱 앞면
2. 8 이웃과 높이 차 > `max_step` — 턱, 보이는 낭떠러지
3. 2 칸 떨어진 칸과 기울기 > 25° — 오를 수 없는 경사
4. 가림 너머 낙차 — 처음 보이는 칸이 `max_step` 보다 낮으면 높은 쪽 칸을 위험으로 (위에서 보면 경사로 옆 가장자리 바로 바깥)

:::{note}
IMU 보정이 필요한 이유: EKF(`state_estimation`) TF 는 roll/pitch 가 0 고정입니다. 그대로 쓰면 10° 경사로 위에서 지면 전체가 기울어 보여 (3) 판정이 틀어졌습니다.
:::

## 기억과 지우기

- 판정 결과는 로봇 둘레 반경 3 m 안에서 **기억**합니다(`radius`, 안 보인 지 60 s 면 잊음 `forget`). 카메라가 다시 안 보는 옆·뒤도 위험이 유지됩니다.
- 판정에는 로봇에서 2 m 안 점만 씁니다(`max_range`, 센서 자체는 4 m 까지).
- `terrain_free` 는 로봇 둘레 원 위의 점들입니다. 비용지도 clearing 이 카메라에서 이 원까지 광선으로 원 안 지형 층을 다 지우고, 바로 위험 칸이 다시 찍힙니다 → 지형 층 = 이 노드의 기억.
- 로봇 중심 0.35 m 안 위험은 버립니다 (몸 끝에서 자기 다리가 잡혀 그 자리 비용지도를 막았음).
- 라이다가 맞힌 칸 0.25 m 안 위험은 내보내지 않습니다 (벽 밑동은 라이다 몫).

## 단계와 다른 로봇

- **탐색 단계(EXPLORE·REGROUP)에서만** 처리합니다. 이동 단계에 앞서가는 Husky 뒷면이 '단차 위험'으로 찍혀 도로를 따라 위험 줄이 전역 비용지도에 남았고, 다음 세트에서 홈으로 가는 계획이 실패했습니다.
- Husky 와 상대 A1 둘레 칸은 기억에 넣지 않습니다 (공동 건물에서 서로를 위험으로 봄).

## Nav2 에 넣기

`terrain_perception/terrain_perception/nav2_patch.py` 와 `tools/terrain_setup.sh` 가 `a1_exploration` 의 Nav2 launch 에 지형 층(ObstacleLayer, marking = `terrain_hazards`, clearing = `terrain_free`)을 넣는 패치 블록을 씁니다. 이 블록은 launch 파일 안에 그대로 있으니 파일을 통째로 덮지 마세요.

## launch 인자

```bash
ros2 launch terrain_perception terrain.launch.py max_step:=0.04 cloud_frame:=auto
```

| 인자 | 기본 | 뜻 |
| --- | --- | --- |
| `max_step` | 0.04 | 넘을 수 있는 최대 단차 (m). CHAMP `swing_height` 와 같게 |
| `cloud_frame` | `auto` | 점구름 프레임. `auto` 면 메시지 header 를 씀 |
