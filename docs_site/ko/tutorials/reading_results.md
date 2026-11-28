# 5. 결과 폴더 읽기

`run_mission.sh` 는 실행마다 `results/<날짜_시각>_set<k>/` 폴더를 만듭니다. 이 튜토리얼에서는 저장소에 들어 있는 통과 실행 `results/20261114_062151_set0` 을 예로 결과를 읽고 그래프로 그립니다.

:::{note}
저장소에 든 예제 실행은 **이전 구조(2026-11-14, 세트 3개 + 공동 건물, 사람 9명)** 의 것입니다. 파일을 읽는 방법은 같지만 세트 수·사람 수·시간은 지금(세트 4개 + 공동 건물, 11명)과 다릅니다. 지금 실행에는 `motion.csv`, `stuck.txt`, `action.log` 도 생깁니다.
:::

## 폴더 구성

| 파일 | 내용 | 먼저 볼 것 |
| --- | --- | --- |
| `summary.txt` | 상태 줄 전체 + 마지막 요약과 `RESULT PASS/FAIL` | ★ 맨 끝 15 줄 |
| `timeline.csv` | 시뮬 10 초마다 한 줄 (단계, 위치, 각 노드 상태, 사람 수, SLAM 오차) | 그래프 그릴 때 |
| `mission_events.txt` | `[MISSION]` `[COMM]` `[HUSKY]` 줄만 뽑은 사건 기록 | ★ 무슨 일이 언제 |
| `mission.log` | 임무 계층 전체 로그 (nav_executor, survivor_detector 등) | 원인 찾을 때 |
| `sim.log`, `slam.log`, `planner.log`, `controller.log`, `terrain.log` | 각 단계 launch 로그 | 기동 실패, Nav2 오류 |

## 1. 판정 보기

```bash
tail -15 results/20261114_062151_set0/summary.txt
```

```text
=== 요약
  TRANSIT  세트 0:      0 ->     57 s (   57 s)
  EXPLORE  세트 0:     57 ->    322 s (  265 s)
  ...
  사람 보고 9 건, 정답 9 명 중 1.0 m 안 일치 9 명: [[10.31, 6.1], ...]
  넘어짐: 없음
  최소 중심 거리: a1_1-husky 1.49 m, a1_2-husky 1.39 m, a1_1-a1_2 0.46 m, 추종 중 Husky 와 최대 2.71 m
  SLAM 공유 자세 최대 오차: a1_1 1.29 m, a1_2 0.34 m
RESULT PASS (마지막 단계 DONE 세트 4, 한도 초과 세트 0, 사람 9/9)
```

| 줄 | 해석 |
| --- | --- |
| 단계별 시간 | 어느 세트가 오래 걸렸는지. 위 실행은 세트 2 탐색이 787 s 로 가장 길었습니다 |
| 사람 보고 | 보고 건수와 정답(Gazebo 배치)에 1 m 안으로 맞은 사람 수. 건수 > 사람 수면 중복·오탐 |
| 최소 중심 거리 | 로봇끼리 가장 가까웠던 거리. A1 몸 길이가 0.6 m 라 0.5 m 아래면 닿았을 수 있음 |
| SLAM 최대 오차 | A1 지도가 얼마나 흘렀는지. 1 m 가까우면 구역·사람 위치가 어긋날 수 있음 |

## 2. 사건 따라가기

```bash
less results/20261114_062151_set0/mission_events.txt
```

여기서 세트 시작·도착, 사람 보고, 통신 전이, Husky 대기를 시간 순으로 볼 수 있습니다. 더 깊은 원인(예: 계획 실패, 추종 실패)은 `mission.log` 에서 로봇 이름과 시각으로 찾습니다.

```bash
grep "a1_1" results/20261114_062151_set0/mission.log | grep -E "실패|홈|도착" | head
```

## 3. 그래프로 그리기

문서의 결과 그래프는 `docs_site/scripts/make_figures.py` 가 `timeline.csv` 에서 그립니다.

```bash
cd ~/ros2_hw && python3 docs_site/scripts/make_figures.py
```

```{figure} ../../images/figures/trajectories.png
:width: 100%

전체 임무의 로봇 경로 (Gazebo 정답 위치, 10 s 간격). 회색 사각형은 각 A1 의 탐색 구역, 별은 사람 정답 위치, 흰 사각형은 Husky 주차점.
```

```{figure} ../../images/figures/slam_error.png
:width: 100%

SLAM 공유 자세 오차. 회색 띠는 이동(TRANSIT), 분홍 띠는 통신 끊김. 세트 2 도착 직전 a1_1 오차가 1.27 m 까지 커졌다가, 도착 때 Husky 기준 보정으로 0.05 m 로 떨어집니다.
```

```{figure} ../../images/figures/people.png
:width: 100%

Husky 가 받은 사람 보고 수. 세트마다 2 명, 공동 건물에서 3 명이 늘어 9 명이 됩니다.
```

다른 실행을 그리려면 스크립트 머리의 `MAIN` 과 `RUNS` 를 그 폴더 이름으로 바꾸세요.

## 4. 여러 실행 비교

이전 구조(fix 단계) 실행을 비교하던 `tools/compare_runs.py` 는 지금 임무 구조의 결과 형식과 다릅니다. 지금 구조에서는 `summary.txt` 의 요약 줄과 `timeline.csv` 를 직접 비교합니다. 누적 기록은 `history/results/runs.csv` 에 있습니다.

파일 형식의 모든 칸은 {doc}`../reference/results_format` 에 정리되어 있습니다.
