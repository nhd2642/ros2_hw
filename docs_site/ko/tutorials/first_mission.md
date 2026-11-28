# 1. 전체 임무 실행하고 지켜보기

이 튜토리얼에서는 전체 임무를 한 번 돌리면서 **각 단계에서 로봇이 무엇을 하는지, 로그의 어디를 보면 되는지** 익힙니다.

:::{admonition} 이 튜토리얼에서 배우는 것
:class: tip
- `run_mission.sh` 의 실행 순서와 결과 폴더
- `mission_watch` 출력 한 줄을 읽는 법
- TRANSIT·EXPLORE·DONE 단계에서 정상 동작의 모습
:::

## 1단계: 실행

```bash
cd ~/ros2_hw/ws && source install/setup.bash
bash src/comm_relay/tools/run_mission.sh
```

처음 몇 줄은 기동 과정입니다.

```text
=== 임무 실행: start_set 0, ROS_DOMAIN_ID 62, 결과 /home/<you>/ros2_hw/results/20261128_005121_set0
두 A1 컨트롤러 활성: 24 s
Nav2 서버 4 개 active: 5 s
start 보냄
```

`start 보냄` 은 스크립트가 action `/husky/run_mission` 으로 임무를 요청했다는 뜻입니다. 직접 요청하려면 `START=0` 으로 띄웁니다 ({doc}`gui_manual_start`).

:::{note}
Nav2 가 안 켜지는 기동 불량이 가끔 있습니다. 그러면 스크립트가 `-> Nav2 만 다시 띄움` 을 출력하고 Nav2 만 2 번까지 다시 띄우며, 그래도 안 되면 전체를 한 번 다시 띄웁니다.
:::

## 2단계: 상태 줄 읽기

임무 요청 뒤로 시뮬 10 초마다 한 줄이 나옵니다. 칸의 뜻은 다음과 같습니다.

| 칸 | 예 | 뜻 |
| --- | --- | --- |
| `t` | `371` | 임무 시작 뒤 시뮬 시간 (s) |
| `phase`, `set` | `EXPLORE 1` | 임무 단계와 세트 번호 (0 = 세트 1, 3 = 세트 4, 4 = 공동 건물) |
| `comm` | `1` | 통신 켬(1) / 끔(0) |
| `husky_x … a1_2_y` | | Gazebo 정답 위치 (평가용, 로봇은 이 값을 모름) |
| `husky` | `ARRIVED` | Husky 주행 상태: `GO` `AVOID` `BLOCKED` `HOLD` `ARRIVED` |
| `a1_1`, `a1_2` | `FOLLOWING` | 각 A1 의 실행기 상태 (`IDLE` `PLANNING` `TURNING` `FOLLOWING` `ESCAPING` `PAUSED` `DONE` …) |
| `a1_1_comm` … | `NORMAL` | 복귀 노드 상태: `NORMAL` `RETURNING` `HOLD` `STOPPING` |
| `people` | `2` | Husky 가 받은 사람 보고 수 |
| `a1_1_slam_err` … | `0.27` | SLAM 공유 자세와 정답 위치의 차 (m) |

## 3단계: TRANSIT — 도로 이동

```{screenshot} transit_follow.png
도로 이동 중: Husky 가 앞서고 A1 두 대가 뒤 좌우 자리를 따라 걷는 모습 (Gazebo 위에서 본 화면).
```

정상이면:

- `husky` 가 `GO` 이고 `husky_x` 가 0.25 m/s 안팎으로 늘어납니다.
- A1 은 Husky 약 1.6~1.9 m 뒤, 좌우 ±0.8 m 근처를 걷습니다 (`a1_1_y` ≈ −0.8, `a1_2_y` ≈ +0.8).
- A1 이 2.5 m 넘게 처지면 Husky 가 `HOLD` 로 기다립니다. 잠깐 나오는 것은 정상입니다.

세트 1 까지는 약 39 초, 세트 사이는 약 49 초(공동 건물 앞은 약 19 초) 걸립니다.

## 4단계: EXPLORE — 건물 탐색

Husky 가 `ARRIVED` 가 되고 두 A1 이 자리에 서면 `phase` 가 `EXPLORE` 로 바뀝니다.

```{screenshot} explore_rviz.png
RViz (tools/vis.rviz 를 a1_1 로 바꾼 것): SLAM 지도, 라이다, 실행기 경로, 사람 표시, 앞 카메라 영상.
```

정상이면:

- A1 상태가 `PLANNING → TURNING → FOLLOWING → IDLE` 을 되풀이합니다 (프런티어 하나에 도착할 때마다).
- 사람을 찾으면 `people` 이 늘고, 터미널(또는 `mission.log`)에 `[MISSION]` 줄로 보고가 찍힙니다.
- 갈 프런티어가 다 떨어지면 A1 이 홈으로 돌아와 `DONE` 이 됩니다. 두 대 다 `DONE` 이면 다음 세트로 갑니다.

세트 하나 탐색은 실행에 따라 약 70~330 초입니다 (지금 설정 7 회). 막힌 목표가 있으면 실패 처리(블랙리스트, 벗어나기, 옮겨 보기)를 거치며 시간이 늘어납니다 — 원리는 {doc}`../concepts/exploration`.

## 5단계: DONE — 요약

공동 건물까지 끝나면 요약과 판정이 출력됩니다.

```text
RESULT PASS (마지막 단계 DONE 세트 5, 한도 초과 세트 0, 사람 11/11, 홈 실패 0)
결과 폴더: /home/<you>/ros2_hw/results/20261128_005121_set0
```

PASS 조건은 다섯 가지입니다: **끝까지 감**, **넘어짐 0**, **탐색 시간 한도(REGROUP) 0**, **사람을 전부 정답 1.0 m 안으로 보고**, **홈 실패 0**. 중복 보고는 판정에 넣지 않으므로, PASS 라도 보고 건수가 11 보다 클 수 있습니다.

## 정리

- 기동 순서와 대기 조건은 `run_mission.sh` 가 처리합니다.
- 상태 줄 하나로 단계·위치·각 노드 상태·SLAM 오차를 함께 볼 수 있습니다.
- 다음 튜토리얼 {doc}`gui_manual_start` 에서는 화면을 켜고 명령을 직접 보냅니다.
