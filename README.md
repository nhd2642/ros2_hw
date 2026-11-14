# disaster_develop

Gazebo Sim 재난 현장 A1 두 대 + Husky 탐색 프로젝트(gazebo_disaster_sim)의 작업 묶음. claude.ai 세션(2026-11-03 ~ 11-12)의 코드·문서·이력 전부.

| 파일/폴더 | 내용 |
|---|---|
| `CLAUDE.md` | Claude Code 가 먼저 읽는 규칙 + 프로젝트 요약·실행법 |
| `HANDOFF.md` | 지금 상태(fix19), 결과 표, 문제 분석, 다음 후보 |
| `docs/mission_design.md` | **지금 구조 (임무 v2, 2026-11-12~)**: 세트 3개 + 공동 건물, Husky 자율주행, A1 추종·탐색·복귀, 노드·토픽, 실행법, 시험 결과 |
| `ws/` | 빌드 작업 공간 (`ws/src` -> `code/ros2_hw_gz_src`). `colcon build --symlink-install` |
| `results/` | `run_mission.sh` 실행 결과 (로그, timeline.csv, summary.txt) |
| `docs/system_structure.md` | 전체 구조·순서도·rqt_graph 형식 노드 그래프·모듈별 원리·맨해튼 가정 (그림은 `docs/img/`) |
| `docs/project_docs/` | claude.ai 프로젝트 문서 21 개 (보행 튜닝부터 실험 계획까지) |
| `history/CHANGELOG.md` | 단계별 이력과 fix 판정 |
| `history/bundles/` | 지금까지 보낸 묶음 zip (install_to_wsl.sh 가 결과 폴더에서 복사) |
| `history/results/runs.csv` | 모든 실행의 누적 요약 |
| `code/ros2_hw_gz_src/` | 최종 코드 사본 (`~/ros2_hw_gz/src`, install_to_wsl.sh 가 복사) |
| `tools/` | compare_runs.py(정답 면적 대비 탐색률 등 비교표), planner_probe/(planner_server 단독 경로 시험), terrain_hazard_count.py(지형 위험 점 세기) |

## 시작하기 (팀원용)

```bash
git clone https://github.com/nhd2642/ros2_hw.git ~/ros2_hw        # WSL 홈에 (Windows 쪽 /mnt/c 아래 말 것: 심볼릭 링크 ws/src 때문)
sudo apt update && sudo apt install -y ros-humble-desktop ros-humble-ros-gz ros-humble-ign-ros2-control \
  ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-robot-localization ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-broadcaster \
  python3-colcon-common-extensions python3-opencv python3-matplotlib python3-pytest
cd ~/ros2_hw/ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install   # 약 2 분
source install/setup.bash && bash src/comm_relay/tools/run_mission.sh                    # 임무 전체 (실제 약 2 시간), 결과는 results/
```
설계·노드·실행 옵션·시험 결과는 `docs/mission_design.md`. 작업 공간은 `ws/` (`ws/src` 는 `code/ros2_hw_gz_src` 로 가는 링크), 사람 인식 모델은 `ws/person_ai_models/`.
`history/bundles/`(옛 묶음 zip)와 `results/` 의 옛 실행 기록은 용량 때문에 저장소에 없고, `install_to_wsl.sh` 는 옛 방식(묶음 복사)이라 더 쓰지 않는다.

읽는 순서: CLAUDE.md -> docs/mission_design.md -> HANDOFF.md (이전 구조 fix19 까지의 기록) -> docs/system_structure.md -> history/CHANGELOG.md.
