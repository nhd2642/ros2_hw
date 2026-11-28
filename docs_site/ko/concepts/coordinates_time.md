# 좌표계와 시간

## 좌표계

```{graphviz} ../../diagrams/frames.dot
```

| 프레임 | 누가 정함 | 쓰는 곳 |
| --- | --- | --- |
| `world` | Gazebo 월드 | `mission.yaml` 의 모든 좌표, `/husky/pose`, `/husky/goal`, 구역 사각형, `pose_fix` |
| `shared_map` | `nav_goal_bridge` (= world 와 같은 방향·원점) | `/shared_map`, `/<ns>/pose_shared`, 프런티어 후보, 목표 배정, `/<ns>/home_shared` |
| `<ns>` 의 `map` | slam_toolbox | 그 로봇의 SLAM 지도, Nav2 전역 계획, `/<ns>/nav_goal_local`, `/<ns>/home_local`, 사람 표시 |
| `<ns>` 의 `odom` | EKF (`state_estimation`) | Nav2 국소 계획, breadcrumb 복귀, 지형 층 |
| `<ns>` 의 `base_footprint` | EKF | 로봇 몸 |

### 로봇마다 TF 가 따로

A1 두 대는 각자 `/<ns>/tf`, `/<ns>/tf_static` 에 TF 를 냅니다(프레임 이름은 둘 다 `map`, `odom`, `base_footprint`). 그래서 RViz 는 로봇마다 하나씩, `/tf:=/<ns>/tf` 로 리맵해서 띄웁니다. 두 로봇을 한 좌표에서 보는 곳은 `shared_map` 입니다.

### map → 월드 정렬

`nav_goal_bridge` 는 각 로봇 map 을 **스폰 자세만큼 평행이동**해서 월드에 놓습니다. 처음 회전은 0 입니다 — map 은 IMU 의 ENU 방향으로 시작하므로 이미 월드와 같은 방향입니다. 세트 도착 때 `pose_fix` 를 받으면 그만큼 더 옮깁니다.

SLAM 지도는 임무 동안 월드에 비해 조금씩 돕니다 (0.5~2.8°). 그래서 탐색 단계로 넘어갈 때마다 정렬의 회전을 odom(IMU) 방향에 맞춥니다 (R1, 2026-11-28: 직전 5 s map→odom 방향 평균, 로봇 위치 중심). 정렬 (x, y, θ, `공유 = R(θ)·map + (x, y)`) 은 `/<ns>/map_anchor` 로 내보내고 `husky_mission` 이 사람 위치 변환에 같은 값을 씁니다.

이 방식은 **스폰 자세를 안다**는 가정에 기대고 있습니다. 시작 위치가 바뀌면 어긋나며, 지도끼리 스캔 매칭으로 맞추는 방식은 아닙니다.

### 좌표 기준은 Husky

A1 의 SLAM 은 흐르지만 Husky 자세 추정(바퀴 속도 × IMU yaw)은 40 m 에 0.15 m 안이었습니다. 그래서 세트마다 A1 지도를 Husky 기준으로 다시 맞춥니다 ({doc}`transit_following`).

### 평가용 정답

`mission_watch.py` 는 Gazebo 정답(`/<이름>/odom/ground_truth`)으로 위치·오차·거리를 잽니다. **로봇 노드는 이 값을 쓰지 않습니다.** 평가 전용입니다.

## 시간

### 시뮬 시간이 기준

모든 노드는 `use_sim_time: true` 입니다. 임무의 한도(탐색 1,500 s, 완료 판정 10 s, 목표 90 s …)는 전부 시뮬 초입니다. RTF 가 0.2~0.4 라서 시뮬 약 1,000 s 임무가 실제로 약 50 분입니다.

예외는 두 곳입니다.

- `return_relay` 의 하트비트 끊김 판정 (벽시계 1 s) — 사용자가 체감하는 통신 단절 시간과 맞추려고.
- `nav_executor` 의 옮겨 보기 최대 시간 `nudge_time` (벽시계 20 s).

### /clock_slow

Gazebo 는 시뮬 스텝마다(1 kHz) `/clock` 을 냅니다. CHAMP 보행 제어(C++)는 이 속도가 필요하지만, rclpy 노드는 `/clock` 을 받는 것만으로 노드마다 CPU 약 20 % 를 썼습니다(파이썬 노드 14 개, RTF 0.36).

`clock_throttle` 노드가 `/clock` 을 받아 **50 Hz `/clock_slow`** 로 다시 냅니다. `mission.launch.py` 는 파이썬 노드 전부를 `/clock:=/clock_slow` 로 리맵해서 띄웁니다. TF 발행도 200 → 50 Hz 로 낮췄습니다.

:::{tip}
새 파이썬 노드를 추가할 때는 `mission.launch.py` 의 `node()` 도우미를 쓰면 리맵이 자동으로 들어갑니다. 직접 띄울 때는 `-r /clock:=/clock_slow` 를 잊지 마세요.
:::
