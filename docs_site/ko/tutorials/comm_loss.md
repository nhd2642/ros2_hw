# 3. 통신 끊김과 복귀 시험

재난 현장에서는 건물 안으로 들어간 로봇과 통신이 자주 끊깁니다. 이 프로젝트는 **통신 장애를 사용자가 직접 켜고 끄는 방식**으로 흉내 냅니다. 이 튜토리얼에서는 탐색 도중 통신을 끊고, A1 이 지나온 길을 되짚어 돌아오는지 확인합니다.

## 무엇이 일어나야 하나

| 시점 | Husky | A1 |
| --- | --- | --- |
| `0` 입력 | 정지 (`/husky/go` = False) | 하트비트가 1 초 끊기면 `RETURNING`: 지나온 길(breadcrumb)을 거꾸로 따라 이번 세트 탐색 시작 위치로 |
| 홈 도착, 여전히 끊김 | 정지 | `HOLD`: 그 자리에 섬 |
| `1` 입력 | 하던 단계 이어 감 | `STOPPING` (1 초 정지) → `NORMAL`, 탐색 재개. 끊긴 동안 찾은 사람은 이때 보고 |

이동 단계(TRANSIT)에서 끊기면 A1 은 되짚지 않고 그 자리에 섭니다.

```{graphviz} ../../diagrams/return_states.dot
```

## 방법 1: 자동 (스크립트)

`COMM_OFF_AT` 은 **탐색 시작 뒤 몇 시뮬 초에 끊을지**, `COMM_OFF_FOR` 는 **몇 초 뒤에 다시 켤지**입니다.

```bash
cd ~/ros2_hw/ws && source install/setup.bash
COMM_OFF_AT=80 COMM_OFF_FOR=60 SETS=1 bash src/comm_relay/tools/run_mission.sh
```

세트 1 탐색 80 초에 `0`, 그 60 초 뒤 `1` 을 보내고, 세트 1 만 하고 끝냅니다. 기다림은 벽시계가 아니라 시뮬 시간(`sim_sleep.py`)으로 재므로 실시간 비율이 바뀌어도 시점이 같습니다.

## 방법 2: 수동

{doc}`gui_manual_start` 의 방법대로 띄운 뒤, A1 이 건물 안에 들어갔을 때 보냅니다.

```bash
ros2 topic pub --once /husky/cmd std_msgs/msg/String "{data: '0'}"
# ... 30~60 초 지켜본 뒤
ros2 topic pub --once /husky/cmd std_msgs/msg/String "{data: '1'}"
```

## 확인하는 법

상태 줄의 `comm` 과 `a1_1_comm`, `a1_2_comm` 칸을 봅니다.

```text
t    phase    set  comm  a1_1       a1_2       a1_1_comm  a1_2_comm
133  EXPLORE  0    1     FOLLOWING  FOLLOWING  NORMAL     NORMAL
144  EXPLORE  0    0     PAUSED     PAUSED     RETURNING  RETURNING
...
184  EXPLORE  0    0     PAUSED     PAUSED     RETURNING  RETURNING
194  EXPLORE  0    0     PAUSED     PAUSED     HOLD       RETURNING
204  EXPLORE  0    1     TURNING    FOLLOWING  NORMAL     NORMAL
```

(통과 실행 `20261114_062151_set0` 의 `timeline.csv` 에서 뽑은 줄입니다.) 끊긴 동안 실행기(`a1_1`, `a1_2` 칸)는 `PAUSED` 로 탐색 목표를 멈추고, 복귀 노드가 `RETURNING` 으로 로봇을 몹니다. 통신이 돌아오면 실행기가 바로 탐색을 다시 시작합니다.

`mission_events.txt` 에는 `[COMM]` 줄로 전이가 남습니다. 노드 쪽 자세한 사건은 `/<ns>/return_events` 토픽에도 나옵니다.

```bash
ros2 topic echo /a1_1/return_events
```

```{screenshot} comm_loss_return.png
통신 0 직후 RViz: 실행기 경로가 사라지고 A1 이 지나온 길을 되짚어 탐색 시작 위치(문 앞)로 돌아오는 모습.
```

## 실제 결과 예

통과 실행 `20261114_062151_set0` 에서 세트 1 탐색 중 144~194 초에 통신을 끊었습니다.

- 두 A1 모두 **1.1 초 뒤** `RETURNING` 으로 바뀌었고 Husky 는 정지했습니다.
- `a1_1` 은 50 초 뒤 홈에 도착해 `HOLD`. 다만 선 자리가 탐색 시작 위치에서 **0.85 m** 떨어진 문 앞이었습니다 — 되짚기가 odom 기준이라 odom 이 흐른 만큼 어긋납니다.
- `1` 뒤 `STOPPING`(1 초) → `NORMAL` 로 탐색을 재개했고, 세트 1 사람 2/2 를 찾았습니다.

## 알아 둘 점

- 기본 설정(`return_drive: direct`)에서는 `return_relay` 가 breadcrumb 를 거꾸로 **직접** 몰아갑니다(0.15 m/s, 제자리 회전 뒤 진행). 되짚을 길 띠(폭 ±0.15 m, 앞 0.5 m) 안에 라이다 점이 최근 두 스캔 모두 3 개 이상 보이면 멈춥니다. `return_drive: auto` 로 바꾸면 Nav2 가 지나온 길을 따르게 할 수 있습니다(시험한 설정은 `direct`).
- 하트비트 판정은 **벽시계** 1 초입니다(시뮬 시간 아님).
- 원리는 {doc}`../concepts/communication_return` 에 있습니다.
