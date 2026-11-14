# A1 전진 보행 대안 조사 (2026-11-04)

배경: CHAMP 기반 A1은 후진은 정상(0.3 m/s까지 선형)이나 전진은 불감대(0.1 명령에서 0)와 상한(약 0.064 m/s)이 있어 파라미터 조정으로 해결 안 됨. vx=0 명령에서 드리프트 0.000 m/s로 "순 추진 편향" 가설도 기각.

## 월드 파일
- gait_test.world 는 gait_test.world.orig 로 원복 완료. 한 번 되돌리면 유지됨.

## 후보 (오픈소스)
| 후보 | A1 지원 | ROS | 시뮬레이터 | 판단 |
|---|---|---|---|---|
| fan-ziqi/rl_sar (Apache-2.0) | O | ROS1 Noetic, ROS 2 Foxy/Humble | Gazebo Classic | 시험 결과 양호(아래) |
| qiayuanl/legged_control | O | ROS1 Noetic, 지원 종료 | Gazebo | Humble 불가 |
| chenci107/A1-QP-MPC-Controller | O | ROS1, Docker | Gazebo(Nvidia GPU) | 전진 보행 확인 문구 없음 |
| legubiao/quadruped_ros2_control | Go2만 | ROS 2 Jazzy | Gazebo Harmonic | A1 아님 |
경진대회 참가팀의 공개 코드는 찾지 못함.

## rl_sar 시험 결과 (사용자 PC, WSL2, Humble, Gazebo Classic, 같은 PC)
- 설치 `~/rl_sar_test`, 실행: `ros2 launch rl_sar gazebo.launch.py rname:=a1` + `ros2 run rl_sar rl_sim`. 키 순서: 0(일어서기) -> 1(걷기 정책) -> W/S/A/D/Q/E, Space(명령 0), R(리셋). 걷기 정책은 `1`을 눌러야 켜짐.
- 사전학습 정책 policy/a1/legged_gym/model.pt (정책 PD kp 20, kd 0.5, action scale 0.125/0.25, 제어 dt 5 ms x4).
- 키보드 결과: 명령 0.1은 전/후진 모두 안 감, 0.2는 찔끔 끊어서 감, 0.3부터 정상 보행(몸통 길이만큼 약 2초). 회전, 후진, 걸음 안정성 모두 좋음.
- 항법 모드(N 키) + `/cmd_vel` 토픽 제어 확인(2026-11-04): `linear.x 0.3` 으로 전진 O, 회전 O (angular.z 0.5 는 느리게 돎). 항법 모드에서는 Q/E 키가 무시되고 토픽만 따름(처음 회전이 안 된 것처럼 보인 이유).
- 해석: 같은 PC의 Gazebo/ODE에서 A1이 전진 보행 가능 => 환경 문제가 아니라 우리 CHAMP 설정(또는 CHAMP 궤적과 이 모델의 조합)이 원인. 정책의 최소 속도 약 0.3 m/s.

## 통합 가능성 (rl_sim 소스 확인)
- 정책 입력: `/cmd_vel`, `/imu`, 관절 상태. `/cmd_vel`, `/imu`, `/joy`, `/pause_physics`, `/reset_world` 는 전역 이름이 코드에 박혀 있고 관절 컨트롤러 command/state 만 네임스페이스(`/<rname>_gazebo/`)를 씀.
- 로봇마다 rl_sim 프로세스를 하나씩 띄우고 토픽 리맵으로 다중 로봇 구성 가능해 보임(미시험). rl_sar launch 는 1대를 stairs 월드에 스폰하는 구조라 우리 월드 + A1 2대 + Husky launch 를 새로 작성해야 함.
- rl_sar A1 모델에는 라이다/카메라 없음 => 센서 추가 필요.

## 다음 단계
1. 프로젝트 launch 구조(재난 월드, Husky) 확인 후 통합 설계.
2. 평지만 시험했음. 붕괴 건물 같은 불규칙 지형에서의 안정성은 미검증.
