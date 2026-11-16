# 진단 도구 쓰기

`comm_relay/tools/` 에는 개발 중 문제를 가르려고 만든 도구가 모여 있습니다. 대부분 **`.sh` 가 필요한 것만 띄우고 `.py` 가 관찰·기록**하는 쌍입니다. 모두 화면 없이 돌고 3~5 분 걸립니다.

```bash
cd ~/ros2_hw/ws/src/comm_relay/tools
```

## 증상별 도구

| 증상 | 도구 | 띄우는 것 | 하는 일 |
| --- | --- | --- | --- |
| 지도가 안 커진다 | `slam_diag.sh` | 시뮬 + SLAM | 관찰 10 s, 360° 회전, 1 m 전진 전후의 지도를 그림으로 남김 |
| 라이다 값이 이상하다 (전부 0.15 m 등) | `lidar_check.sh` | 시뮬을 세 가지 렌더링 방식으로 | 스캔·깊이 점구름 한 장씩 읽어 비교 |
| Nav2 계획이 실패한다 | `nav_probe.sh` | 시뮬 + SLAM + Nav2 | 5/25/50 s 에 지도·비용지도·계획 결과 기록 |
| Nav2(DWB) 가 안 움직인다 | `nav_debug.sh` | 시뮬 + SLAM + Nav2 + 지형 | 목표 3 개를 주고 명령·odom·DWB 평가 기록 |
| 지형 위험이 이상하다 | `terrain_check.sh` | 시뮬 + SLAM + Nav2 + 지형 | 점구름 프레임, 경사로 가장자리 위험 위치 |
| 걷기·odom 이 이상하다 | `walk_check.sh` | 시뮬 | 평지 0.2 m/s 8 s, 제자리 0.5 rad/s 6 s 뒤 정답과 odom 비교 |
| 저속 명령을 못 따른다 | `lowspeed_check.sh` | 평지 시뮬, A1 한 대 | 속도마다 3 s 가속 + 6 s 측정 |

## 떠 있는 시스템에 붙여 쓰는 도구

임무가 돌고 있을 때 다른 터미널에서 (같은 `ROS_DOMAIN_ID` 로):

```bash
python3 map_snapshot.py a1_2 /tmp/a1_2_map.png "세트 2 탐색 중"   # SLAM 지도 + 지금 스캔을 PNG 로
python3 costmap_check.py        # 비용지도에 라이다 장애물이 찍히는지
python3 tf_check.py             # 국소 비용지도 라이다 층이 하는 TF 변환을 그대로 해 봄
python3 sim_sleep.py 120        # 시뮬 시간으로 120 s 기다림 (스크립트용)
```

DWB 가 0 명령을 내는 순간을 기록하려면 Nav2 추종 서버를 `A1_DWB_DIAG=1 ros2 launch a1_exploration nav2_controllers.launch.py` 로 띄워 `/<ns>/evaluation` 을 켜고 `dwb_diag.py` 로 기록합니다 (궤적 점수라 무거워서 기본은 끔).

## 경로 계획만 따로 시험 (planner_probe)

저장소 최상위 `tools/planner_probe/` 는 **planner_server 하나만** 띄워 저장된 지도로 경로를 구합니다. 팽창 반경 같은 계획 설정을 시뮬 없이 비교할 때 씁니다.

| 파일 | 역할 |
| --- | --- |
| `fake.py` | `map.npz` → `/a1_1/map`, 가짜 `/clock`·TF |
| `params.yaml.in` | `RADIUS` 를 바꿔 쓰는 planner 설정 틀 |
| `client.py` | 스폰 → 5 곳 `ComputePathToPose`, 길이·벽 여유 출력 |

:::{note}
`tools/compare_runs.py`, `tools/terrain_hazard_count.py` 와 planner_probe 는 이전 구조(건물 한 채, fix 단계) 때 만든 도구입니다. 지금 임무 구조에서는 `summary.txt`·`timeline.csv` 로 비교합니다.
:::
