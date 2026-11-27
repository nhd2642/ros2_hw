# 인수인계

## ★★ 2026-11-27 인계 (가장 최신 — 이 절부터 읽는다. 아래 11-21 절은 그 전 상태)

### 0. 먼저 알 것 (중요)
- **main 은 11-21 발표 영상 버전 (`ccb992e`) 그대로.** 11-26 ~ 27 작업은 브랜치 `claude/project-code-review-structure-c9b240` 에 커밋·push 됨 (main 에 안 합침).
  로컬 작업 폴더는 git 워크트리 `~/disaster_develop/.claude/worktrees/project-code-review-structure-c9b240`. `~/disaster_develop` (main 체크아웃) 을 열면 이 변경이 **안 보인다.**
  다른 PC: `git clone -b claude/project-code-review-structure-c9b240 https://github.com/nhd2642/ros2_hw.git ~/ros2_hw` (WSL 홈에) 후 `cd ~/ros2_hw/ws && colcon build --symlink-install`.
- 새 세션은 위 워크트리 폴더를 작업 폴더로 연다 (아래 '연결할 폴더'). 커밋은 사용자가 요청할 때만, 커밋 메시지에 Claude Co-Authored-By 줄 넣지 않음.
- 빌드: `ws/src -> ../code/ros2_hw_gz_src` 심볼릭 링크. `cd ws && colcon build --symlink-install`. 파이썬 스크립트는 symlink 라 고치면 바로 반영
  (scan_ready_relay.py 는 dual_slam.launch.py 가 `ws/src/.../scripts` 에서 직접 실행).
- 사람 인식 모델: `ws/person_ai_models/yolov4-tiny/` (24 MB, 저장소에 넣음). MobileNet-SSD 파일은 지움.

### 1. 연결할 폴더 (새 세션)
| 폴더 | 왜 |
|---|---|
| `/home/nhd2642/disaster_develop/.claude/worktrees/project-code-review-structure-c9b240` (**필수, 작업 폴더**) | 코드 `code/ros2_hw_gz_src`, 빌드 `ws/`, 문서 `docs/`·`docs_site/`, 실험 결과 `results/` (git 밖), 이 문서 |
| `/home/nhd2642/disaster_develop` (선택) | main 체크아웃. Claude 메모리(`~/.claude/projects/-home-nhd2642-disaster-develop/memory/`)가 이 경로 기준이라 같은 프로젝트로 묶임 |
| `C:\Users\fllno\ros2_hw_ppt_materials` = `/mnt/c/Users/fllno/ros2_hw_ppt_materials` (발표 자료 만들 때만) | 11-21 에 모은 발표 자료 (rqt_graph 3 장, mission_design, 실험 기준, 배치도, 영상). **주의: 그 안 `3_interfaces/README_인터페이스_설계.md` 는 'custom msg 없음, JSON' 기준이라 지금은 틀림** -> `docs_site/ko/reference/interfaces.md` 로 바꿔야 함 |
- 영상: Google Drive 폴더 'ros2_hw 발표 영상 (2026-11-21)' (`set4_x8.mp4`, `mission_x20.mp4`). 11-21 버전 영상이다 (지금 코드 아님).
- 다른 워크트리 `.claude/worktrees/moveit-documentation-8088ed` 는 예전 문서 사이트 작업, 이번 일과 무관.

### 2. 사용자 규칙 (메모리에도 있음)
- 답은 한국어. 계획에 없던 변경이 필요하면 **먼저 보고하고 확인** 받기. 커밋은 요청 때만, Claude 공동 저자 줄 없음.
- docs_site 는 로컬 빌드로만 확인, 배포하지 않음 (결과 확정 뒤).
- 시뮬은 한 번에 하나. 실험 규칙: 한 묶음 = 동작 변경 하나, 판정 기준을 실행 **전에** `results/<실험>_기준.txt` 에 적고, 바꾼 동작이 몇 번 작동했는지 먼저 센다.
  망가진 실행(기동 실패 등)은 폴더에 `ABORTED.txt` 를 두고 다시 돌린다. 개인정보(이메일 등)는 외부 서비스에 보내지 않음.

### 3. 이 세션 전체 흐름 (2026-11-18 ~ 11-27)
11-18 ~ 11-21 (자세히는 아래 11-21 절, `docs/experiments/*_기준.txt`):
- 코드·구조 검토 -> Nav2 복귀 경로 삭제(직접 복귀만) -> 정량 지표 도구(summary, stuck_report, motion.csv) 정리.
- 사용자 방향 (11-18): **장애물이 복잡할수록 강건성 어필** -> 장면은 그대로 두고 보행·Nav·SLAM 쪽을 고친다. 어려운 세트 4 (x 40~48) 추가.
- 실험: N2 DWB 후진 기각, **N1 MPPI 채택**, N3 Smac 기각, G2 옆걸음 채택했다가 되돌림 (카메라가 길을 안 봄), S1 SLAM 재생 실험 중단 (재생이 재현 안 됨).
- 발표용 2D 애니 `tools/mission_anim.py`, slam_replay 버그 수정. 사양 확인: Core Ultra 5 226V, WSL2 7.5 GB, RTF 0.2~0.3.
- 11-21: main 에 합침 (ccb992e), HANDOFF 갱신, 영상 2 개 Drive 업로드, 발표 자료 폴더, 평가표용 인터페이스는 'JSON 설명으로 대체' 로 일단 결정.

11-26 사용자 요청 **'수정 사항 두 개'** (이 버전 = 발표 영상 버전 ccb992e 가 기준선):
1. 평가표에 맞게 custom msg 로 변경 + 코드 전체 검토.
2. A1 이 실내에서 탐색한 곳을 재탐색·또 재탐색 (빙글빙글) 하는 문제 + 보행이 너무 느려 보임 -> 한 번에 잘 탐색 + 보행 속도 높이기.

그 뒤 한 실험 (순서대로, 판정 전부 `docs/experiments/` 와 `results/` 의 `*_기준.txt`):
- **E1 (채택)**: 세 변경을 묶어서 (사용자 결정).
  - `mission_interfaces` 패키지 신설: msg MissionState·Zone·SurvivorReport·SurvivorList, srv SetComm·GetSurvivors, action RunMission
    (`/husky/run_mission`, `/husky/set_comm`, `/husky/get_survivors`). JSON 문자열 토픽 3 개와 `/husky/cmd` 문자열 명령을 대체. 노드 7 개 + 도구 수정.
    문서 `docs_site/ko|en/reference/interfaces.md` 신설.
  - 재탐색 방지: 탐색 목표 도착 시 nav_executor -> `/exploration_visited`, frontier_detector 가 그 세트 동안 그 1.0 m 안 칸을 프런티어로 안 셈 (칸 단위).
    프런티어 최소 5 -> 10 칸.
  - MPPI vx_max 0.2 -> 0.35 m/s (`nav2_controllers.launch.py`). CHAMP 보행 자체(걸음 주기 등)는 안 바꿈. 회전 wz 0.5 그대로.
  - 벤치마크 6/6 PASS, 탐색 합 2061 -> 1281 s (-38 %), 재방문 17.6 -> 14.0 %. 전체 임무 PASS 1/2 (FAIL = set4 북쪽 사람을 라이다 거리 확인에서 버림).
- **D1 (인식, 부분 통과)**: MobileNet-SSD -> YOLOv4-tiny (OpenCV DNN 416). 옆모습 사람 인식 0/21 -> 21/21.
  같은 D1 안에서 `survivor_logic.locate` 를 직선거리 대신 깊이와 비교 (시야 40° 에서 1.3 배 차 버그), 기준 키 1.75 -> 2.0.
  전체 2 회: 1 회 FAIL (경사로 SLAM 4.3 m 어긋남 — 인식과 무관), 1 회 PASS 11/11 (1094 s).
- **경사로 시험장** (`comm_relay/tools/ramp_check.sh/.py`, `sim.launch.py a1_1_pose:=x,y,yaw`): 세트 1 남쪽 경사로에서 정해진 걷기·회전 순서, 미끄러짐·SLAM 오차 측정 (한 번 약 6 분).
  원인 발견: slam_toolbox Humble 은 odom **이동 거리로만** 스캔을 거름 -> 제자리 회전 중 처리 스캔 0~1 -> 경사 위 0.8~1.0 m 미끄러짐을 못 봄. 속도는 원인 아님.
- **S3 (채택)**: `slam_params.yaml minimum_travel_distance 0.2 -> 0.0`. 시험장 회전 중 처리 스캔 0~1 -> 30~33, 최대 오차 1.12~1.26 -> 0.07~0.16 m.
  전체 2 회 PASS 2/2 (11/11, 홈 실패 0, 979 / 1046 s, SLAM 최대 0.50/0.44, 0.37/0.28 m).
- **검토 + 계획** (`~/.claude/plans/joyful-churning-knuth.md`, 승인): S3 뒤 나빠 보인 두 가지
  - Nav2 기동 실패: S3 탓 아님 (S3 전에도 있음, Fast-DDS lifecycle 응답 시간 초과). `run_mission.sh` 에 'Nav2 만 다시 띄우기' (2 번까지, 실패 로그 `planner.failN.log` 보관),
    cleanup 무늬 `send_goal.*/husky/run_mission` (+ -9) 수정, `mission_watch.py` SetComm 재시도, `sim.launch.py` a1_1_pose 검사, ramp_check MODE=arc 삭제.
    기동 시험 `DUR=30` x 10: 10/10 첫 시도 성공 (`results/20261127_boot_test`).
  - 공동 건물 SLAM 오차 (0.43/0.34 m, 이전 MPPI 중앙값 0.22) -> S4.
- **S4 (기각)**: SLAM 에 스캔을 필요할 때만 넣기 (`a1_exploration/scripts/scan_ready_relay.py`, `/<ns>/scan_ready` 는 slam_toolbox 만 씀).
  - 첫 안 (odom 이동 0.18 m / 회전 20° 마다): s4_1..3 무효 (서 있으면 0 개라 SLAM 시작 못 함 -> gate_period 5 s 추가),
    s4b_1..3 기준 통과했지만 **회전 중 최대 오차 0.35~0.48 m** (S3 0.06~0.16). 원인: 경사 위 회전이 멈추고 몸이 0.4 m 미끄러지는 4.5 s 동안 odom 은 0.03 m 만 봐서 스캔 0 개.
    이 기준으로 돌던 전체 임무 s4full_1/2 는 중단·무효.
  - 지금 안 **S4c**: `/<ns>/cmd_vel` (CHAMP 입력) 이 0 이 아니었던 때부터 `gate_hold` 1 s 안이면 다 보냄, 명령 0 으로 서 있으면 `gate_period` 5 s 마다 하나.
    시험장 3 회는 통과했지만 전체 임무 2 회 **FAIL / FAIL** -> **S4 기각, S3 유지, 코드 되돌림** (아래 6 절, `docs/experiments/S4_기준.txt`).

### 4. 발표 영상 버전(ccb992e) 대비 지금 (2026-11-27 판단)
| | 영상 버전 (N1 MPPI, 전체 2 회) | 지금 (E1+D1+S3 = 커밋된 최종, 전체 2 회 s3full) |
|---|---|---|
| 전체 임무 PASS | 1/2 (놓친 사람 1) | **2/2 (11/11)** |
| 전체 시간 (시뮬) | 1910 / 1582 s | **979 / 1046 s (약 -40 %)** |
| 세트별 탐색 s | 228/405/400/348/324, 169/262/221/304/397 | 115/135/136/175/211, 136/109/143/170/257 |
| 재방문 비율 | 25.4 / 18.7 % | 15.2 / 12.7 % (E1 전체 기준, S3 실행은 안 셈) |
| 추종 평균 속도 | 0.15~0.19 m/s | 0.22~0.29 m/s |
| 사람 인식 | SSD, 옆모습 놓침 | YOLOv4-tiny, 옆모습 인식 |
| 경사로 SLAM | 회전 중 못 봄 (시험장 최대 1.2 m) | 0.07~0.16 m |
| SLAM 최대 (전체) | 0.72/0.48, 0.25/0.23 m | 0.50/0.44, 0.37/0.28 m (같은 범위) |
| 평가표 인터페이스 | JSON 문자열 + /husky/cmd | custom msg·srv·action |
| 넘어짐 / 홈 실패 | 0 / 0 | 0 / 0 |
- 나빠진 것은 없음. Nav2 기동 실패는 예전부터 있던 것이고 이제 자동으로 다시 띄움. 단 실행 수가 적다 (각 2 회).
- 요청 대비: 1 번 (custom msg) 완료. '코드 전체 검토' 는 인터페이스·탐색 경로 위주였고 (E1 보고에 명시), 그 뒤 /code-review 2 번 (11-27) 으로 작업 트리 전체를 봄.
  2 번 재탐색은 완료 (재방문 줄고 탐색 시간 -38~45 %). '보행 속도' 는 Nav 최대 속도를 올린 것이고 CHAMP 걸음새 자체는 그대로.
- D1·S3·S4 는 요청 2 개를 확인하다 나온 실패(사람 놓침, 경사로 SLAM)를 사용자가 골라 고친 것. **S4 는 요청 범위 밖의 다듬기**:
  S3 로 이미 PASS 2/2 이고 SLAM 최대 오차도 영상 버전 범위 안이라, S4 를 안 해도 요청은 충족. S4 는 전체 임무 0/2 로 기각 -> 최종은 S3.

### 5. 열린 과제
1. (S4 는 끝남, 기각) 공동 건물 SLAM 오차를 더 줄이려면 다른 방법을 찾아야 함. S4c 에서 '갈 목표 없음 90 s'·폐경로가 늘어난 원인은 확인 안 함 (서 있을 때 SLAM 갱신이 멈추는 것으로 추정).
2. 문서·커밋은 끝남 (S4 기각까지 반영, 브랜치에 커밋·push). main 에 합칠지는 사용자 결정.
3. 놓친 사람이 남는 곳: set4 남쪽 L 자 복도 안쪽 방 (46.6, -1.9) — S4c 2 회 모두 a1_1 이 1.1 m 까지 가고도 놓침 (S3 2 회는 찾음). 라이다 거리 확인 '무시' 가 실행당 6~54 번.
4. 발표 자료 폴더의 인터페이스 설명 갱신 (custom 인터페이스로), 새 영상이 필요하면 `tools/mission_anim.py <결과 폴더> --speed=20` (s3full_1b 추천).
5. 코드 리뷰 남은 것 (11-27): Nav2 대기 60 s 로 줄임 (nit), `survivor_logic.locate` 가 깊이로 기준을 잡고 묶음은 직선거리로 함 (nit, 인식 동작이 바뀌므로 따로 결정).
6. 예전부터 남은 것 (11-21 절): 폐경로가 유일한 길을 닫음, 카메라 시야 밖 사람, slam_replay 재현 안 됨, MPPI 계산 부하, 통과율 표본 수.

### 6. S4c 시험장 결과
S4c 시험장 결과 (20261127_180051/_180700/_181402_ramp_s4c_1..3): 모두 통과
   a. turn360 처리 스캔 31~34 (평균 32) ✓   b. stand 0, stand_flat 2 (S3 10) ✓
   c. turn360 중 최대 오차 0.13~0.21, 평균 0.17 m ✓ (S3 0.06~0.16 보다 약간 큼, s4b 0.35~0.48 보다 작음)
   d. 회전 직후 0.02 m, stand_flat 끝 0.00~0.02 m ✓   e. 넘어짐 0 ✓
   -> 전체 임무 2 회 (*_set0_s4cfull_1,2) 로.
전체 임무 2 회: 20261127_182034_set0_s4cfull_1 FAIL (10/11, 1352 s, SLAM 최대 1.03/0.70 m), 20261127_194212_set0_s4cfull_2 FAIL (8/11, 1160 s, 0.24/0.24 m). 홈 실패 0.
  S3·E1·D1 전체 5 회에는 0 이던 '갈 수 있는 목표 없이 90 s' 4/6 번, 폐경로 줄 39/58 개 -> S4 기각, S3 유지, scan_ready_relay.py 를 커밋 ccb992e 상태(= S3 동작)로 되돌림.

### 7. 자주 쓰는 명령
```bash
cd ~/disaster_develop/.claude/worktrees/project-code-review-structure-c9b240
source /opt/ros/humble/setup.bash && source ws/install/setup.bash
bash code/ros2_hw_gz_src/comm_relay/tools/run_mission.sh clean        # 남은 프로세스 정리
TAG=base bash code/ros2_hw_gz_src/comm_relay/tools/ramp_check.sh      # 경사로 시험장 (results/<시각>_ramp_<TAG>/ramp.txt)
OUT=$PWD/results/$(date +%Y%m%d_%H%M%S)_set0_<이름>_1 bash code/ros2_hw_gz_src/comm_relay/tools/run_mission.sh   # 전체 임무, summary.txt 에 RESULT
DUR=30 bash code/ros2_hw_gz_src/comm_relay/tools/run_mission.sh       # 기동만 확인
cd code/ros2_hw_gz_src/comm_relay && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -p no:cacheprovider   # 단위 시험 (67 통과, terrain_perception 18)
```
- 결과 읽기: `summary.txt` (RESULT, 사람, SLAM 공유 자세 최대 오차, 멈춤), `mission_events.txt`, `timeline.csv`, `motion.csv`, `stuck.txt`, `action.log`.
- scratch 분석 스크립트 (`/tmp/claude-1001/.../scratchpad/`) 는 재부팅 때 지워질 수 있음 — 필요한 것은 위 도구로 다시 만든다.

---


## ★ 2026-11-21 인계 (지금 상태 — 이 절부터 읽는다)

### 지금 상태
- 임무: Husky 가 도로를 따라 세트 4 곳 + 공동 건물을 돌고, A1 두 대가 따라가 건물마다 탐색·사람 찾기·복귀 (사람 11 명).
  세트 4 (x 40~48) 는 배치 규칙을 **일부러 어기는** 어려운 장면 (폭 1.2 m L 자 복도, 1.0~1.2 m 틈의 기둥·상자 방, 라이다 아래 낮은 잔해).
- 최종 설정 = 기준선 + **국소 컨트롤러 DWB → MPPI** (`a1_exploration/launch/nav2_controllers.launch.py`, 앞 0~0.2 m/s, 후진·옆걸음 없음).
- 결과: 어려운 장면 벤치마크에서 멈춤 397 → 56 s, 홈 실패 1 → 0. MPPI 전체 임무 4 회 PASS 2 (FAIL 둘 다 사람 1 명 놓침), 홈 실패·넘어짐 0.
- 자세한 결과·원인·한계: `docs/mission_design.md` 5 절 (같은 내용 `docs_site/ko|en/results/`).
  실험마다 실행 전에 적은 판정 기준과 결과: `docs/experiments/*_기준.txt` (N1 채택, N2·N3 기각, G2 채택했다가 되돌림, S1 중단, Final = 전체 임무).

### 남은 과제 (우선순위 순, 근거는 mission_design.md '남은 한계')
1. **폐경로가 유일한 길을 닫음** — 같은 자리 추종 실패 2 번이면 `nav_executor` 가 그 자리를 폐경로로 닫는데, 세트 4 남쪽처럼 안쪽 방으로 가는 유일한 복도면 방을 포기한다
   (`docs/experiments/Final_기준.txt` N1 2 회). 후보: 닫은 자리를 일정 시간 뒤 다시 열기, 또는 프런티어가 그 너머에만 남으면 한 번 더 시도.
2. **카메라 시야 밖 사람 놓침** — 사람 인식은 카메라 ±52° 안이어야 한다. 탐색 끝은 라이다 지도 기준이라 '카메라로 본 곳' 을 안 센다.
   후보: 세트 끝내기 전에 사람 후보 쪽(방 구석 등)으로 한 바퀴 돌기, 또는 카메라 시야 커버리지 지도.
3. **SLAM 오프라인 재생이 재현 안 됨** — `comm_relay/tools/slam_replay.py` 로 같은 bag·같은 설정 8 번 → 최대 오차 0.97~4.03 m. 원인을 찾아야 SLAM 파라미터 실험(S1·S2) 가능.
   후보: 재생마다 받은/처리한 스캔 수, `tf_static` 받은 시각, 정렬 시점을 찍어 어디서 갈리는지.
4. **MPPI 계산 부하** — controller 주기(10 Hz, 벽시계) 놓침 경고가 실행당 수천 번. 결과는 노트북 CPU (Core Ultra 5 226V, WSL2 7.5 GB, 카메라 소프트웨어 렌더링) 기준이라 더 빠른 PC 에서 다시 확인 필요.
5. **통과율** — 전체 임무 PASS 2/4. 몇 번에 한 번 통과하는지 말하려면 더 돌려야 한다 (한 번 약 1.6 시간).

### 하지 말 것 (이미 해 보고 기각)
- DWB 후진 허용 (N2): 앞뒤 진동·odom 흐름 → 홈 실패 3.
- Smac 2D 플래너 (N3): 이득 없음.
- MPPI 옆걸음 (G2, Omni vy 0.1): 이동의 45~50 % 를 몸이 비스듬히 걸어 카메라가 길을 안 봐 사람을 놓침.

### 작업 규칙 (이번에 쓴 것)
- 한 묶음 = 동작 변경 하나. 판정 기준을 실행 **전에** `results/<실험>_기준.txt` 에 적고, 바꾼 동작의 작동 횟수를 먼저 센다.
- 벤치마크: `SETS=1 BAG=1 bash src/comm_relay/tools/run_mission.sh <k>` 를 k = 1 (세트 2), 3 (세트 4), 4 (공동 건물) × 2 회. 세트 하나 약 25 분, 한 번에 시뮬 하나만.
- 기동 불량(Nav2·TF 안 뜸)으로 망가진 실행은 폴더에 `ABORTED.txt` 를 두고 다시 돌린다.
- 멈춤 원인은 `motion.csv` + `stuck_report.py` (summary 에 '원인 보행 / Nav'). 발표 영상은 `tools/mission_anim.py <결과 폴더> --speed=20`.

### 받아서 실행
```bash
git clone https://github.com/nhd2642/ros2_hw.git ~/ros2_hw   # main 에 최종본이 합쳐져 있음. WSL 홈에 (/mnt/c 아래 말 것)
cd ~/ros2_hw/ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash
bash src/comm_relay/tools/run_mission.sh            # 전체 임무 (실제 약 1.6 시간), 결과 ~/ros2_hw/results/
SETS=1 bash src/comm_relay/tools/run_mission.sh 3   # 세트 4 만
```
apt 패키지는 `README.md`. 실험 결과 폴더(`results/`, 36 GB)와 발표 영상은 용량 때문에 저장소에 없다.

---

> **2026-11-12 방향 전환 (임무 v2)**: 건물 1채·Husky 고정 구조에서, Husky 가 도로를 따라 세트 3개와 공동 건물을 차례로 도는 구조로 바뀌었다.
> 지금 구조·실행법·시험 결과는 `docs/mission_design.md`. 아래 본문은 그 전(fix19)까지의 기록이며, 6 절 문제 분석과 7 절 실수 목록은 지금도 참고한다.
> 작업 위치는 `code/ros2_hw_gz_src` (빌드 `ws/`), 원본 `~/ros2_hw_gz` 는 건드리지 않았다. 묶음(zip)·apply.sh 방식은 더 쓰지 않는다.

이 폴더(`~/disaster_develop`)는 claude.ai 세션(2026-11-03 ~ 11-12)에서 한 작업 전부의 사본이다. 아래 본문은 claude.ai 프로젝트의 `HANDOFF_NEXT_SESSION.md`(2026-11-12 01:20) 그대로이고, 맨 위 "Claude Code 에서 달라지는 점"만 덧붙였다.

## Claude Code 에서 달라지는 점
- **묶음(zip) + 사용자 실행 방식이 필요 없다.** 지금까지는 클라우드에서 고친 파일을 `bundles\record17_gz_fixNN.zip` 으로 만들어 사용자가 apply.sh 로 덮었다. Claude Code 는 WSL 안에서 `~/ros2_hw_gz/src` 를 직접 고치고 빌드·시험·실행할 수 있다.
  - 고치기 전에 git 으로 지금 상태를 남겨 둘 것 (`~/ros2_hw_gz/src` 는 git 이 아닐 수 있음 -> `~/disaster_develop/code/ros2_hw_gz_src` 사본이 fix19 시점 기준).
  - 팀원 launch(`a1_exploration/launch/nav2_*.launch.py`)에는 terrain_setup.sh 등이 넣은 패치 블록이 있다. 통째로 덮지 말고 필요한 줄만 고친다.
- **실행은 길다** (시뮬 600 s = 실제 35~40 분). `gazebo_test_mission.sh` 를 백그라운드로 돌리고, 끝나면 결과 폴더(`/mnt/c/Users/fllno/comm_relay_results/<날짜_시각>_mission`)를 읽는다. 같은 시각에 두 실행을 돌리지 않는다 (ROS_DOMAIN_ID 는 스크립트가 바꾸지만 CPU 가 모자람).
- **클라우드 전용 내용은 무시**: 본문 3 절의 "클라우드 `/home/claude/gz/ws`", "device_stage_files" 는 claude.ai 세션 환경 이야기다. Claude Code 에서는 로컬 파일을 바로 읽고 `tools/compare_runs.py` 를 바로 돌린다.
- 프로젝트 문서 21 개 사본: `docs/project_docs/`. 실험 표: `docs/project_docs/experiment_plan_20261111.md`. 시스템 구조: `docs/system_structure.md`.
- 다음 묶음 이름을 쓸 일이 있으면 fix20 부터.

---

**이 문서 하나로 시작한다.** 함께 읽을 것: `claude/experiment_plan_20261111.md` (실험 규칙·fix15~19 결과 표), `claude/review_fix7_to_fix15_20261111.md` (fix7~15 재검토·실수·과한 수정). 시스템 구조·순서도·노드 그래프: Claude 문서 "A1 재난 탐색 시뮬레이션 시스템 구조" (https://claude.ai/artifact/AWcSAL8acEwV2X1oy1cgDL, fix15 기준). 사용자는 한국어로 대화한다.

## 0. 새 세션의 첫 할 일
1. 이 문서 + `claude/experiment_plan_20261111.md` 를 읽는다.
2. 사용자 PC 는 **fix19** 적용 상태 (= fix15 동작 + fix18 전역 팽창 0.70 + fix19 옮겨 보기 연장). 실행 중인 것 없음.
3. 사용자에게 다음 실험 하나를 고르게 한다 (8 절 후보 A~C, 추천 A). 고른 것만 묶음(fix20)으로 만든다: 단위 시험 + 클라우드 노드 단독 확인 -> bundles 에 넣고 한 줄씩 명령 -> "끝" 하면 compare_runs 로 판정.

## 1. 프로젝트 목표 (사용자 확정 2026-11-09)
1. 각 A1 이 자기 쪽을 자율 탐색하며 자체 맵 생성
2. 사람(마네킹) 발견 -> Husky 에 신호 + 자체 맵 표시. 통신이 끊겨 있으면 모았다가 복구 때 보냄, 발견 시 1 초 정지 후 탐색 계속
3. 자기 구역을 다 탐색하면 원위치(스폰) 복귀
4. Husky 통신 불량은 사용자 조작(on/off), `/husky/comm_cmd`
5. SLAM frontier + Nav2, Husky 는 구동 안 함
6. Husky 가운데, 남(a1_1)/북(a1_2) 구역 분담 (경계 y=0 고정)
+ Gazebo Sim(Fortress) + ROS 2 Humble. 수정은 "다른 맵에서도 통하는 일반적 방향". 사용자 의견(2026-11-11): 폐경로·못 지나가는 틈은 블랙리스트를 굳이 남기지 말고 재탐색만 안 하게.

## 2. 작업 규칙
- 프로젝트 CLAUDE.md: 가정은 말하고, 단순하게, 필요한 곳만, 검증 기준을 정한 뒤 확인. 읽은 숫자만, 추측은 추측이라고.
- **실험 규칙 (2026-11-11 확정)**: 한 묶음 = 동작 변경 하나. 같은 설정 2 회 DUR=600 (DWB_DIAG 끔). 비교는 미션 시뮬 300/600 s 시점, 900 s 실행도 600 s 까지. 판정 기준은 실행 전에 적는다. 알려진 무관한 고장으로 망가진 실행은 '무효/영향받음' 표시 후 1 회 더.
- **사용자 PC 전에**: 단위 시험 + 바뀐 노드를 클라우드에서 단독 실행 확인 (예: 지형 노드 + fake_depth, planner_server + 실행 지도). 못 하면 못 했다고 말한다.
- 주 지표 (compare_runs.py): 구역별 정답 면적 대비 탐색률 %, 넘어짐, 멈춤 합 s, 홈 도착, 맞은 마네킹 수/3 (0.6 m 안), 도착 수 + 고장별: 경사로 가장자리 구역(|x| 0.6~1.5, y -3.2~-1.7) 멈춤 s·실패 수, 계획 실패 수, 폐경로 거부 수.
- 결과는 사용자가 "끝"이라고 하면 결과 폴더를 직접 읽는다 (device_stage_files 로 runs.csv, events.csv, progress/map_0300s·0600s.npz, console.txt 를 가져와 클라우드에서 compare_runs). 명령은 **한 줄씩**. GitHub 은 사용자가 직접.
- URDF/xacro 에 주석·비ASCII 금지. 묶음 apply.sh 는 파일 통째 덮기 대신 팀원 launch 는 sed + 개수 확인(fix18 방식).

## 3. 파일 위치
| 무엇 | 어디 |
|---|---|
| 결과 폴더 | Windows `C:\Users\fllno\comm_relay_results` = WSL `/mnt/c/Users/fllno/comm_relay_results` (기기 연결 폴더 `~/mnt/comm_relay_results`) |
| 실행마다 | `<날짜_시각>_mission\` : console.txt, events.csv (t 벽시계 / sim 미션 시뮬 초), goals.csv, timeseries.csv, map.npz, progress/map_XXXXs.npz (t = 미션 시뮬 초, 공유 지도 = 월드 정렬), run_map.png, progress.png, mission.log.gz, controller.log.gz, planner.log.gz, costmap_diag/. 누적 `runs.csv` |
| 묶음 | `bundles\` record17_gz_fix8 ~ fix19 (fix9·fix16·fix17 은 되돌린 것) |
| 비교 도구 | `comm_relay/tools/compare_runs.py <결과 폴더> <월드 파일> [실행 ...]` (fix16 묶음으로 사용자 PC 에도 있음). 정답 = 월드 벽 상자(높이 0.3 m 이상) + Husky 0.99x0.67 + 마네킹 0.5x0.4, 건물 안(|x|<4.85, |y|<6.5), 스폰에서 4 방향으로 닿는 자기 구역 칸: 남 58.7 / 북 58.8 m² |
| 사용자 WSL | Gazebo Sim `~/ros2_hw_gz`, 시험 로그 `~/comm_relay_tests/` |
| 클라우드 | `/home/claude/gz/ws` (새 세션에는 없음 -> cloud_tools.zip README 로 재구축 약 10 분). 이번 세션 확인 방법: 지형 노드 = `tools/fake_depth.py` + `python -m terrain_perception.terrain_node`, 플래너 = planner_server 단독 + 실행 map.npz 를 /a1_1/map 로 발행 + 가짜 /clock·TF + ComputePathToPose(use_start) |

## 4. 지금 시스템 (사용자 PC = fix19)
- Nav2: NavFn(A*, tolerance 0.25, allow_unknown), footprint 0.6x0.32, **전역 팽창 0.70 (fix18)**, 국소 팽창 0.35, cost_scaling 3.0. DWB: max 0.2 m/s, critics Oscillation·BaseObstacle·GoalAlign·PathAlign·PathDist·GoalDist (RotateToGoal 뺌, fix10), 도착 yaw 3.14, min_speed_xy 0.03·min_speed_theta 0.05 (정지 궤적 제외, fix12).
- nav_executor 복구 층: 벗어나기, 옮겨 보기(같은 자리 계획 실패 3 번, **0.3 m 뒤에도 로봇 칸 비용 >= 128 이면 트인 거리까지, fix19**), 0 명령 감시 8 s(fix12 뒤 사실상 안 쓰임), 기울기 가드 25°/1 s(작동 기록 0), 반복 실패 차단(fix13, 목표 기준 반경 1 m 3 번 — 거의 안 쓰임), 폐경로(fix14/15: 같은 자리 추종 실패 2 번 -> 닫고 1 m 물러남, 닫힌 자리 0.8 m 를 지나는 목표 안 감, 실행 내내), 원위치 계획 실패 시 지나온 길 역추종(fix13), reached_ttl 60 s.
- 지형: hazard_pad 0 (fix16 되돌림).

## 5. 결과 (600 s 시점, compare_runs)
| 묶음 | 실행 | 탐색률 남/북 % | 멈춤 s | 경사로 멈춤 s / 실패 | 계획 실패 | 넘어짐 | 홈 | 마네킹 |
|---|---|---|---|---|---|---|---|---|
| fix15 (기준) | 094803 / 110450 | 93.5/83.1, 96.2/90.2 | 218, 26 | 131/7, 1/1 | 4, 0 | 0 (094803 은 694 s 에 1) | 0 | 2, 2 |
| fix16 기각 | 115109 / 131712 | 76.0/92.6, 87.7/89.4 | 122, 118 | 31/12, 13/11 | - | 0 | 0 | 1, 3 |
| fix17 미채택 | 151526 / 172908 (+155419 영향받음) | 88.4/92.3, 61.5/76.9 | 21, 186 | 0/0, 95/6 | -, 6 | 0 | 1, 0 | 3, 0 |
| fix18 조건부 채택 | 182441 / 192008 | 93.4/79.9, 93.4/85.2 | 105, 46 | 0/0, 0/1 | 11, 4 | 0 | 0 | 3, 2 |
| fix19 판정 불가 | 224919 / 234107 (+223443 무효) | 80.4/79.5, 69.2/95.7 | 126, 82 | 75/1, 0/1 | 3, 1 | 1 (224919 310 s), 0 | 0 | 2, 1 |

- fix19: 두 실행 모두 '옮겨 보기' 가 한 번도 일어나지 않음 (mission.log 에 '옮겨 보기 (' 0 줄) -> 효과 판정 불가. 해는 없는 것으로 보고 둔 상태 (사용자 확인 필요: 두거나 되돌리기).
- 남쪽 하락의 실제 원인 (fix19 와 무관): 224919 = a1_1 경사로 동쪽 가장자리 넘어짐 310 s. 234107 = a1_1 폐경로 거부 8 번 + IDLE 51 %, 남쪽 69.2 %.

## 6. 문제 분석 (2026-11-12 기준, 빈도순)
1. **경사로 옆 가장자리 넘어짐** — 20+ 실행 중 4 번, 전부 경사로(ramp_south_a, x ±0.95, 10°) 옆 가장자리 (fix11 서 (-0.98,-2.95), fix15 동 (1.04,-2.67), fix19 동 (1.03,-3.01)). 공통: FOLLOWING 중 0.18~0.2 m/s, 넘어지기 직전까지 기울기 < 25° (기울기 가드로 못 막음).
   - 224919 새 사실: 직전 20 s 동안 경사로 위를 남동쪽으로 비스듬히(yaw -45~-66°) 걸었고 **0.6 m 안 지형 위험 칸 0 개, 가장 가까운 위험 1.7 m** -> 지형 카메라는 앞만 봐서 옆 낭떠러지를 위험으로 못 남김. fix18(위험 둘레 비용)은 위험 칸이 있어야 작동.
   - 앞선 넘어짐 2 번은 위험 칸이 0.19~0.33 m 옆에 있었는데도 감 (fix18 전).
2. **폐경로 거부 과다** — 094803 a1_2 13 번, 131712 a1_1 11 번, 234107 a1_1 8 번 (IDLE 51 %, 남쪽 69 %). 닫힌 자리 반경 0.8 m 를 지나는 경로가 많아 넓은 구역이 통째로 막힘. 닫힌 자리가 실제 막힌 곳이었는지 확인 안 함.
3. **팽창 영역 안 시작 함정** — 계획 실패 + 로봇 칸 비용 170~253, 옮겨 보기 0.3 m 로 못 빠짐 (155419 a1_2 87 % IDLE). 약 4/20 실행. fix19 로 대응했지만 아직 발동 확인 없음.
4. **기동 실패** — 시작 직후 goal_assigner 'No safe reachable goal', 두 로봇 0 m, 120 s 에 모니터가 끝냄 (065634, 223443). 약 2/25 실행. 원인 미조사.
5. **홈 복귀가 거의 없음** — 600 s 안 홈 도착은 151526 a1_1 한 번뿐. 완료 판정이 안 나거나(남은 프런티어 오감), 이르게 나거나(172908 61 % 에서 완료), 홈 계획이 실패(182441 a1_2 3.6 m 앞).
6. **북쪽 변동** — fix18 뒤 북쪽 79.9·85.2 (기준 83.1·90.2). 계획 실패는 a1_2 (-0.8, 4.3)·Husky 옆 (0.7, 0.9) 에서 주로.
7. 사람 위치 오차 — 발견의 절반가량이 가장 가까운 마네킹에서 1 m 넘게 떨어짐 (거리 추정 또는 오탐). 마네킹 3 명 다 맞힌 실행은 3 번.
8. RTF 0.19~0.28 (600 s = 실제 약 35~40 분).

## 7. 지난 실수 (요약, 자세히는 review_fix7_to_fix15)
- 1 회 실행으로 결론, 한 묶음에 여러 변경, 클라우드 확인 없이 동작 변경 전달 (fix9·fix13#2·fix14 버그를 사용자 실행으로 발견), 새 로직 단위 시험 없음, fix11 넘어짐 놓침, 정답 면적 없이 '맵 끝' 판단, '0 명령 정지 0' 이 갇힘(제자리 회전)을 가린 지표 문제. 11/11 부터 규칙으로 고침.
- fix19 는 효과를 보려면 발동 횟수(로그)를 판정 기준에 넣었어야 했음 -> 다음부터 "바꾼 동작이 몇 번 작동했나" 를 먼저 센다.

## 8. 다음 후보 (사용자가 하나 고름)
- **A (추천). 경사로 옆 가장자리 인식** — 넘어짐 원인. 설계안 (하나만 골라 실험):
  1. 지형 위험 기억 강화: 한 번 본 위험 칸을 지금 반경 밖에서도 실행 내내 유지 (경사로를 정면으로 볼 때 본 가장자리를 비스듬히 갈 때도 씀). 가장 단순. 처음 접근이 비스듬하면 못 막음.
  2. 몸 기울기 변화로 가장자리 감지: 롤이 급히 커지면(예: 1 s 에 8° 이상) 그 쪽 옆을 위험 칸으로 찍고 멈춤. 감지가 늦을 수 있음.
  3. 경사 위 이동 제한: 피치/롤 > 5° 일 때 경로 방향과 경사 방향 차가 크면(비스듬히) 속도 제한 또는 정면 정렬. 넘어짐이 드물어 2 회로 판정 어려움.
  판정 지표: 경사로 가장자리 구역 멈춤 s·실패, 넘어짐, 남쪽 탐색률. 넘어짐은 드물어 '경사로 위 비스듬 이동 시간'·'가장자리 0.3 m 안 접근 횟수' 같은 대리 지표를 먼저 compare_runs 에 추가할 것.
- B. 폐경로 거부 과다: 닫힌 자리 반경 0.8 -> 0.4 m 또는 '지나감' 판정을 경로 전체가 아니라 닫힌 자리 칸을 실제로 밟는 경우만. 지표: 폐경로 거부 수, IDLE %, 탐색률.
- C. 기동 실패 원인 조사 (동작 변경 없이 로그 분석 먼저: goal_assigner 'No safe reachable goal' 때 지도·자세 상태).
- 그 밖: fix19 유지/되돌림 결정, 구역 경계 margin(경로 0.15 m, 실행당 1~3 번 거부), 북서 구석, 사람 위치 오차, 일반화(구역 분할 경로 거리 기반, 지도 합침 스캔 매칭, 4 방향 BFS -> 8 방향; 맨해튼 월드 가정 6 곳은 시스템 구조 문서 참고), 복구 층 정리(0 명령 감시·기울기 가드·fix13 반복 실패 차단은 사실상 안 쓰임).

## 9. 다음 실험 명령 (사용자 PC, 묶음을 만든 뒤)
```
cd ~ && cp /mnt/c/Users/fllno/comm_relay_results/bundles/record17_gz_fix20.zip ~/ && unzip -o record17_gz_fix20.zip
bash ~/record17_gz_fix20/apply.sh
DUR=600 bash ~/ros2_hw_gz/src/comm_relay/tools/gazebo_test_mission.sh mission
```
(마지막 줄 2 번. 600 s 는 실제 약 35~40 분.)
