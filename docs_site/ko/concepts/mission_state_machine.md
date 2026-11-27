# 임무 상태기계

임무의 진행은 `husky_mission` 노드 하나가 정합니다. 이 페이지는 단계, 전이 조건, 단계마다 로봇이 하는 일, 그리고 통신 규칙을 설명합니다.

```{graphviz} ../../diagrams/mission_states.dot
```

## 세트

세트 순서는 `mission.yaml` 의 `sets` 순서 그대로입니다: `set1`, `set2`, `set3`, `common`. 코드와 로그의 세트 번호 `k` 는 0 부터 셉니다 (0 = 세트 1, 3 = 공동 건물).

각 세트에는 Husky 주차점(`park`), 로봇별 탐색 구역 사각형(`zones`), 사람 수(`people`)가 있습니다.

## 단계별 동작

| 단계 | Husky | A1 | 끝나는 조건 |
| --- | --- | --- | --- |
| **IDLE** | 정지 | 정지 | 사용자 `start` |
| **TRANSIT k** | 세트 k 주차점으로 주행. A1 이 자기 자리에서 2.5 m 넘게 처지면 서서 기다리고 1.5 m 안으로 오면 다시 감 | Husky 뒤 자기 자리(뒤 1.6 m, 좌우 0.8 m)를 따라감 | Husky 도착 + A1 둘 다 자리 0.6 m 안 (또는 Husky 도착 60 s 경과) |
| **EXPLORE k** | 정차. 사람 보고 집계 | 자기 구역 프런티어 탐색. 갈 프런티어가 30 s 없으면 탐색 시작 위치로 복귀 → `set_done` | 두 A1 모두 `set_done = k` |
| **REGROUP k** | 정차 | 남은 탐색을 접고 탐색 시작 위치로 복귀 | 둘 다 복귀 또는 300 s |
| **DONE** | 정지 | 정지 | — |

EXPLORE 가 `explore_timeout`(1,500 s)을 넘으면 REGROUP 이 됩니다. 판정에서 REGROUP 이 한 번이라도 나오면 FAIL 입니다.

## 탐색 시작 위치 (홈)

TRANSIT 이 끝나 EXPLORE 로 넘어가고 2 초 뒤, 각 A1 의 그때 위치를 **이번 세트의 홈**으로 기록하고 `/<ns>/home_shared` 로 내려 줍니다. A1 은 탐색을 마치면 스폰이 아니라 이 홈으로 돌아옵니다. 그래서 다음 TRANSIT 은 모두가 도로 위에서 출발합니다.

## 세트 완료 규칙

- 세트 완료 = **A1 두 대 모두** 홈 복귀 (`/<ns>/set_done`).
- 찾은 사람이 설정 수보다 적으면 '미발견 n' 으로 남기고 진행합니다. 지도가 끝났으면 더 볼 곳이 없으므로 기다리지 않습니다.
- 복귀를 8 번 시도해도 못 돌아오면 그 자리에서 세트 완료로 보고합니다. 그러면 다음 TRANSIT 에서 Husky 가 그 A1 을 기다리며 섭니다.

## 공동 건물

공동 건물(`common`)은 두 A1 의 구역이 **같은 사각형**(x 40~54, y −6~6)입니다.

- 합친 지도에서 프런티어를 뽑으므로 한쪽이 본 곳은 다른 쪽 후보에서도 빠집니다.
- 목표 배정은 두 목표가 3 m 보다 가까우면 벌점을 줘 서로 떨어진 프런티어를 고릅니다.
- 출발 자리가 문에서 4 m 떨어져 문 너머 지도가 듬성듬성하면 목표가 안 나옵니다. 그때는 프런티어에 가장 가까운 '닿을 수 있는 칸'으로 먼저 다가갑니다(접근 목표, 로봇이 구역 밖일 때만).

## 사용자 요청

액션·서비스로 보냅니다 (형식과 예는 {doc}`../reference/interfaces`). `husky_mission` 을 띄운 터미널에 `start` / `0` / `1` 을 쳐도 같습니다.

| 요청 | 효과 |
| --- | --- |
| action `/husky/run_mission` (`num_sets`) | 임무 시작 (IDLE → TRANSIT). `num_sets` 개 세트를 마치면 DONE, 결과로 생존자 목록. 취소하면 그 자리에서 DONE |
| service `/husky/set_comm` `enabled: false` | 통신 끔: 하트비트 중지. A1 은 이번 세트 홈으로 복귀(이동 단계면 그 자리에 섬), Husky 는 정지 |
| service `/husky/set_comm` `enabled: true` | 통신 켬 (기본): 하던 단계를 이어 감. 끊긴 동안 찾은 사람은 이때 보고 |
| service `/husky/get_survivors` | 지금까지 확인한 생존자 (전체 또는 세트별) |

## 통신 하트비트

`husky_mission` 은 통신이 켜져 있는 동안 로봇마다 `/<ns>/comm_status` (Bool) 를 계속 냅니다. 이것이 끊기는 것이 곧 '통신 단절'입니다. 위치(거리·벽)로 통신을 계산하지 않고 사용자 조작으로 정한 것은 프로젝트 목표(2026-11-09 확정)입니다. 로봇 쪽 반응은 {doc}`communication_return`.

## /mission/state

1 Hz 로 나가는 `mission_interfaces/msg/MissionState` (latched) 입니다. 다른 노드는 이것만 보고 단계를 압니다.

```text
phase: EXPLORE
set_index: 1
set_name: set2
zones:
- {robot: a1_2, x_min: 16.0, x_max: 24.0, y_min: 1.5, y_max: 6.5}
- {robot: a1_1, x_min: 16.0, x_max: 24.0, y_min: -6.5, y_max: -1.5}
comm: true
```

| 칸 | 뜻 |
| --- | --- |
| `phase` | `IDLE` `TRANSIT` `EXPLORE` `REGROUP` `DONE` |
| `set_index`, `set_name` | 세트 번호(0 부터)와 이름 |
| `zones` | 로봇별 구역 (월드 m). EXPLORE·REGROUP 일 때만 채워지고 그 밖에는 빈 배열 |
| `comm` | 통신 켬 true / 끔 false |

같은 주기에 TRANSIT 이면 `/husky/goal`(주차점, `world`), EXPLORE·REGROUP 이면 `/<ns>/home_shared`(홈, `shared_map`)도 함께 냅니다. 홈은 EXPLORE 로 넘어가고 2 초 뒤의 A1 위치입니다.
