# 통신·복귀·명령 중재

## 통신 모델

통신 장애는 **사용자 조작**(`0` / `1`)으로 흉내 냅니다. 거리·벽으로 계산하지 않습니다(프로젝트 목표, 2026-11-09).

- `husky_mission` 은 통신이 켜져 있는 동안 로봇마다 `/<ns>/comm_status` (Bool) 하트비트를 냅니다.
- `0` 이면 하트비트를 멈추고 Husky 를 세웁니다(`/husky/go = False`).
- 로봇 쪽은 하트비트가 **벽시계 1 초** 끊기면 단절로 봅니다. 시뮬 실시간 비율과 무관하게 같은 시간으로 판정하려고 벽시계를 씁니다.

## 복귀 — return_relay

```{graphviz} ../../diagrams/return_states.dot
```

| 상태 | 뜻 |
| --- | --- |
| `WAIT` | 하트비트나 odom 을 아직 한 번도 못 받음 → 아무 명령도 안 냄 |
| `NORMAL` | 통신 정상. 지나온 위치를 0.1 m 간격 breadcrumb 로 쌓음 (최대 5,000 개) |
| `RETURNING` | 하트비트 1 s 끊김 → breadcrumb 를 역순으로 되짚어 복귀 (즉시 복귀 정책) |
| `HOLD` | breadcrumb 를 다 되짚고 홈에 왔는데 여전히 단절 → 정지 명령 유지 |
| `STOPPING` | 복귀 중 통신 복구 → 1 s 정지 명령 후 `NORMAL` |

### 되짚기 주행

- breadcrumb 와 추종은 로봇 자신의 **odom 프레임** 하나로만 합니다. SLAM 보정이 튀어도 길이 흔들리지 않지만, odom 이 흐른 만큼 홈에서 어긋납니다 (마지막 실행 0.85 m).
- 복귀는 직접 주행: 다음 breadcrumb 쪽으로 0.35 rad 넘게 틀어져 있으면 먼저 제자리 회전(0.4 rad/s), 아니면 0.15 m/s 로 전진하며 방향을 맞춥니다.
- **길 위 장애물**: 걸을 때만, 앞으로 되짚을 길 띠(폭 ±0.15 m, 앞 0.5 m) 안에 라이다 점이 최근 두 스캔 모두 3 개 이상이면 멈추고, 비면 다시 갑니다. breadcrumb 는 몸이 실제로 지나간 자리라 그때는 비어 있었으므로, 새로 생긴 장애물만 걸립니다 (상대 A1 은 라이다에 작게 보여 못 걸릴 수 있습니다, {doc}`../results/limitations`).
- 0.5 m 넘는 odom 점프는 breadcrumb 를 끊습니다.
- 이동 단계(TRANSIT)에서 끊기면 되짚지 않고 그 자리에 섭니다.

### 실행기와의 관계

`return_relay` 는 상태를 `/<ns>/comm_mode` (transient local)로 냅니다. `nav_executor` 는 이것이 `RETURNING`/`HOLD` 면 탐색 목표를 멈추고(`PAUSED`), `NORMAL` 로 돌아오면 다시 시작합니다.

### 왜 STOPPING 이 필요한가

CHAMP 에는 `cmd_vel` 타임아웃이 없어 **마지막 명령으로 계속 걷습니다.** 팀원 원본은 복귀 끝에 발행만 멈췄기 때문에 로봇이 계속 걸어갔습니다. 그래서 명시적으로 0 명령을 1 s 냅니다.

## 명령 중재 — cmd_vel_arbiter

```{graphviz} ../../diagrams/cmd_vel_arbiter.dot
```

로봇마다 CHAMP 의 `/<ns>/cmd_vel` 에 쓰는 쪽을 **하나로** 만듭니다.

| 우선순위 | 입력 | 보내는 노드 |
| --- | --- | --- |
| 4 | `/<ns>/cmd_vel_teleop` | 사람 조종 |
| 3 | `/<ns>/cmd_vel_hold` | `survivor_detector` (사람 발견 1 s 정지) |
| 2 | `/<ns>/cmd_vel_return` | `return_relay` |
| 1 | `/<ns>/cmd_vel_nav_test` | Nav2 controller_server, `nav_executor` 의 회전·벗어나기 |
| 0 | `/<ns>/cmd_vel_follow` | `a1_follower` |

- 0.5 s 안에 들어온 입력 중 우선순위가 가장 높은 것을 20 Hz 로 냅니다.
- 들어온 입력이 하나도 없으면 **0 명령**을 냅니다 (CHAMP 타임아웃 없음 문제를 여기서 막음).
- 지금 이긴 입력 이름을 `/<ns>/cmd_vel_source` 로 냅니다.
- `/<ns>/cmd_vel` 에 다른 발행자가 보이면 경고합니다.

판단 로직은 `arbiter_logic.Arbiter` 에 있고 단위 시험됩니다.
