# 빠른 시작: 첫 임무 실행

{doc}`installation` 을 마쳤다면, 이 페이지의 명령 한 줄로 **시뮬레이션부터 임무 끝까지** 전부 실행할 수 있습니다.

## 한 줄로 실행

```bash
cd ~/ros2_hw/ws && source install/setup.bash
bash src/comm_relay/tools/run_mission.sh
```

`run_mission.sh` 는 다음 순서로 모든 것을 띄우고, 준비될 때마다 다음 단계로 넘어갑니다.

```{graphviz} ../../diagrams/launch_order.dot
```

| 단계 | 하는 일 | 기다리는 조건 |
| --- | --- | --- |
| 1 | 월드 + Husky + A1 두 대 (`sim.launch.py`) | 두 A1 의 관절 컨트롤러가 활성 (최대 180 s) |
| 2 | 로봇별 SLAM (`dual_slam.launch.py`) | 5 s |
| 3 | Nav2 계획·추종 서버 | 서버 4 개 `active` (60 s 안에 안 되면 Nav2 만 2 번까지 다시 띄움) |
| 4 | 지형 인식 (`terrain.launch.py`) | 3 단계와 같이 |
| 5 | 임무 계층 (`mission.launch.py`) | 10 s |
| 6 | 임무 요청: action `/husky/run_mission` (`num_sets` = `SETS`, 0 = 끝까지) | — |

그 뒤 `mission_watch.py` 가 **시뮬 10 초마다 한 줄**씩 상태를 출력합니다.

```text
t,phase,set,comm,husky_x,husky_y,a1_1_x,a1_1_y,a1_2_x,a1_2_y,husky,a1_1,a1_2,...
21 TRANSIT 0 1 0.79 -0.00 -1.54 -1.37 -1.21 0.82 GO IDLE IDLE NORMAL NORMAL 0 0.05 0.07
31 TRANSIT 0 1 3.29 -0.00  1.46 -0.80  1.47 0.81 GO IDLE IDLE NORMAL NORMAL 0 0.05 0.04
...
```

:::{important}
전체 임무는 **실제 시간 약 50 분** 걸립니다 (시뮬 약 900~1,050 초, 실시간 비율 0.3 안팎, 개발 PC 기준). 처음에는 {doc}`../tutorials/partial_sets` 처럼 **공동 건물만** 돌려 보면 훨씬 짧습니다.

```bash
SETS=1 bash src/comm_relay/tools/run_mission.sh 4
```
:::

## 화면으로 보고 싶다면

기본은 화면 없이(headless) 돌립니다. Gazebo 화면을 켜려면 `GUI=1` 을 붙이세요.

```bash
GUI=1 bash src/comm_relay/tools/run_mission.sh
```

```{screenshot} start_formation.png
GUI=1 로 띄운 Gazebo 화면: 출발점의 Husky 와 그 뒤 A1 두 대 (a1_1 앞 작은 벽 포함).
```

RViz 로 지도·라이다·카메라·경로를 보려면 로봇마다 RViz 를 하나씩 띄웁니다. 로봇마다 TF 가 `/<ns>/tf` 로 따로 있어서, 저장소의 템플릿 `tools/vis.rviz` 의 `/NS/` 를 로봇 이름으로 바꿔 씁니다.

```bash
T=~/ros2_hw/ws/src/comm_relay/tools
sed "s#/NS/#/a1_1/#g; s#XPOS#0#" $T/vis.rviz > /tmp/vis_a1_1.rviz
rviz2 -d /tmp/vis_a1_1.rviz --ros-args -r /tf:=/a1_1/tf -r /tf_static:=/a1_1/tf_static -p use_sim_time:=true
```

이 화면에는 그 로봇의 SLAM 지도(`/a1_1/map`), 라이다, 로봇 모델, 실행기 경로(`/a1_1/nav_executor/path`), 사람 표시(`/a1_1/survivor_markers`), 앞 카메라 영상이 나옵니다 (Fixed Frame `map`). `a1_2` 도 같은 방법으로 띄웁니다.
:::{note}
`run_mission.sh` 는 실행마다 `ROS_DOMAIN_ID` 를 무작위(20~79)로 정합니다. 다른 터미널에서 `ros2 topic` 이나 RViz 를 쓰려면 스크립트가 처음에 출력한 값과 같게 맞추세요.

```bash
export ROS_DOMAIN_ID=<출력된 값>
```
:::

## 결과 확인

끝나면 마지막에 이런 요약이 나옵니다 (`20261128_005121_set0_r1full_1`).

```text
=== 요약
  TRANSIT  세트 0:      0 ->     39 s (   39 s)
  EXPLORE  세트 0:     39 ->    111 s (   72 s)
  ...
  사람 보고 11 건, 정답 11 명 중 1.0 m 안 일치 11 명
  넘어짐: 없음
RESULT PASS (마지막 단계 DONE 세트 5, 한도 초과 세트 0, 사람 11/11, 홈 실패 0)
```

결과 폴더는 `~/ros2_hw/results/<날짜_시각>_set<k>/` 입니다. 액션 결과(생존자 위치 목록)는 그 안 `action.log` 에 남습니다. 무엇이 들어 있는지는 {doc}`../tutorials/reading_results` 를 보세요.

## 중간에 멈추기

`Ctrl+C` 로 멈추면 스크립트가 띄운 프로세스를 모두 정리합니다. 혹시 남은 프로세스가 있으면:

```bash
bash src/comm_relay/tools/run_mission.sh clean
```

## 다음 단계

- {doc}`../reference/interfaces` — 임무 요청·통신 끄기·생존자 조회를 직접 보내기
- {doc}`../tutorials/gui_manual_start` — 화면을 보며 직접 시작하기
- {doc}`../tutorials/comm_loss` — 통신 끊김과 복귀 시험
- {doc}`../concepts/architecture` — 지금 띄운 것들이 어떻게 이어지는지
