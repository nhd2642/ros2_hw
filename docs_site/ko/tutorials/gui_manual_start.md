# 2. 화면 보며 직접 시작하기

이번에는 Gazebo 화면을 켜고, 자동 `start` 없이 **사용자가 직접** 임무를 시작하고 통신을 조작합니다. 두 가지 방법이 있습니다.

| 방법 | 언제 |
| --- | --- |
| A. `run_mission.sh` + 토픽으로 명령 | 가장 간단. 기동은 스크립트가, 명령은 사용자가 |
| B. 각 단계를 터미널 6 개로 직접 실행 | 노드 하나를 고쳐 가며 시험할 때, `husky_mission` 터미널에 직접 입력하고 싶을 때 |

## 방법 A: 스크립트로 띄우고 토픽으로 명령

```bash
cd ~/ros2_hw/ws && source install/setup.bash
GUI=1 START=0 bash src/comm_relay/tools/run_mission.sh
```

`START=0` 이면 기동까지만 하고 기다립니다. 다른 터미널에서 같은 `ROS_DOMAIN_ID` 로 맞추고 명령을 보냅니다.

```bash
source ~/ros2_hw/ws/install/setup.bash
export ROS_DOMAIN_ID=<스크립트가 출력한 값>
ros2 topic pub --once /husky/cmd std_msgs/msg/String "{data: start}"
```

같은 토픽에 `0`, `1` 을 보내면 통신을 끄고 켭니다.

```bash
ros2 topic pub --once /husky/cmd std_msgs/msg/String "{data: '0'}"   # 통신 끔
ros2 topic pub --once /husky/cmd std_msgs/msg/String "{data: '1'}"   # 통신 켬
```

## 방법 B: 터미널 6 개로 직접 실행

각 터미널에서 먼저 `source ~/ros2_hw/ws/install/setup.bash` 를 하고, 모든 터미널의 `ROS_DOMAIN_ID` 를 같게 둡니다.

::::{tab-set}

:::{tab-item} 1. 시뮬
```bash
ros2 launch disaster_sim sim.launch.py
```
월드 + Husky + A1 두 대. `a1_2` 가 서기까지 약 30 초 기다립니다.
:::

:::{tab-item} 2. SLAM
```bash
ros2 launch a1_exploration dual_slam.launch.py
```
:::

:::{tab-item} 3. Nav2
```bash
ros2 launch a1_exploration nav2_planners.launch.py
# 다른 터미널
ros2 launch a1_exploration nav2_controllers.launch.py
```
로그에 `Managed nodes are active` 가 나올 때까지 기다립니다.
:::

:::{tab-item} 4. 지형
```bash
ros2 launch terrain_perception terrain.launch.py
```
:::

:::{tab-item} 5. 임무 계층
```bash
ros2 launch comm_relay mission.launch.py husky:=false
```
`husky:=false` 면 `husky_mission` 만 빼고 띄웁니다.
:::

:::{tab-item} 6. Husky 임무
```bash
ros2 run comm_relay husky_mission --ros-args \
  -p config:=$(ros2 pkg prefix disaster_sim)/share/disaster_sim/config/mission.yaml \
  -p use_sim_time:=true -r /clock:=/clock_slow
```
이 터미널에 `start`, `0`, `1` 을 입력하고 Enter.
:::
::::

```{screenshot} terminal_start.png
husky_mission 터미널에 start 를 입력한 직후의 로그 ([MISSION] TRANSIT 세트 1 …).
```

:::{warning}
`husky_mission` 은 `config` 파라미터로 `mission.yaml` 경로를 받아야 합니다. launch(`husky:=true`)로 띄우면 자동으로 넣어 주지만, `ros2 run` 으로 띄울 때는 위처럼 직접 넘기세요. 이 경로(터미널 직접 입력)는 개발 중에 따로 돌려 보지 않았으므로, 문제가 있으면 방법 A 를 쓰세요.
:::

## Gazebo 화면 둘러보기

```{screenshot} gazebo_gui.png
GUI=1 Gazebo 창: 왼쪽 엔티티 트리(husky, a1_1, a1_2, person_*), 가운데 3D 화면, 아래 실시간 비율(RTF).
```

- 아래쪽 **RTF**(Real Time Factor)가 0.2~0.4 면 정상입니다. 화면을 켜면 더 느려집니다.
- 엔티티 트리에서 `a1_1` 을 오른쪽 클릭 → *Follow* 하면 카메라가 따라갑니다.
- 화면을 켠 실행은 시험 횟수가 적습니다. 정식 결과는 화면 없이 돌리세요.

## RViz 로 내부 보기

RViz 로 지도·라이다·카메라·경로를 보려면 로봇마다 RViz 를 하나씩 띄웁니다. 로봇마다 TF 가 `/<ns>/tf` 로 따로 있어서, 저장소의 템플릿 `tools/vis.rviz` 의 `/NS/` 를 로봇 이름으로 바꿔 씁니다.

```bash
T=~/ros2_hw/ws/src/comm_relay/tools
sed "s#/NS/#/a1_1/#g; s#XPOS#0#" $T/vis.rviz > /tmp/vis_a1_1.rviz
rviz2 -d /tmp/vis_a1_1.rviz --ros-args -r /tf:=/a1_1/tf -r /tf_static:=/a1_1/tf_static -p use_sim_time:=true
```

이 화면에는 그 로봇의 SLAM 지도(`/a1_1/map`), 라이다, 로봇 모델, 실행기 경로(`/a1_1/nav_executor/path`), 사람 표시(`/a1_1/survivor_markers`), 앞 카메라 영상이 나옵니다 (Fixed Frame `map`). `a1_2` 도 같은 방법으로 띄웁니다.

## 다음 단계

{doc}`comm_loss` 에서 탐색 중 `0` 을 보내 복귀 동작을 시험합니다.
