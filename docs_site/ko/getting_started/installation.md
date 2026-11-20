# 설치

이 페이지는 **빈 Ubuntu 22.04 (WSL2 포함)** 에서 시뮬레이션을 빌드할 수 있는 상태까지 안내합니다. 약 20~30 분 걸립니다.

:::{note}
이 프로젝트는 **Windows 11 + WSL2 (Ubuntu 22.04)** 에서 개발·시험했습니다. 일반 Ubuntu 22.04 PC 에서도 같은 순서로 됩니다.
:::

## 준비물

| 항목 | 권장 |
| --- | --- |
| OS | Ubuntu 22.04 (Jammy), 또는 Windows 11 의 WSL2 Ubuntu 22.04 |
| CPU | 8 코어 이상. 시뮬레이션은 CPU 를 많이 씁니다 (RTF 0.3 안팎) |
| 메모리 | 16 GB 이상 |
| 디스크 | 15 GB 이상 여유 (ROS 2 + Gazebo + 빌드) |
| GPU | 필요 없음. 센서 렌더링은 소프트웨어 GL 로 합니다 |

## 1. ROS 2 Humble 설치

ROS 2 가 아직 없다면 [공식 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 대로 저장소를 추가합니다.

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 2. 필요한 패키지 설치

ROS 2 데스크톱, Gazebo 연동(`ros_gz`), ros2_control, Nav2, slam_toolbox, 그리고 사람 인식용 OpenCV 를 한 번에 설치합니다.

```bash
sudo apt update && sudo apt install -y ros-humble-desktop ros-humble-ros-gz ros-humble-ign-ros2-control \
  ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-robot-localization ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-broadcaster \
  python3-colcon-common-extensions python3-opencv python3-matplotlib python3-pytest
```

각 패키지가 하는 일:

| 패키지 | 쓰는 곳 |
| --- | --- |
| `ros-humble-ros-gz`, `ros-humble-ign-ros2-control` | Gazebo Sim (Fortress) 실행, 토픽 다리, A1 관절 제어 |
| `ros-humble-ros2-control`, `ros-humble-ros2-controllers` | CHAMP 가 내는 관절 명령을 시뮬 관절에 전달 |
| `ros-humble-robot-localization` | A1 의 EKF (odom) |
| `ros-humble-slam-toolbox` | 로봇별 2D SLAM |
| `ros-humble-nav2-bringup` | 경로 계획(NavFn)·추종(MPPI, `ros-humble-navigation2` 에 포함된 `nav2-mppi-controller`) 서버 |
| `python3-opencv` | MobileNet-SSD 사람 인식 (OpenCV DNN) |
| `python3-matplotlib`, `python3-pytest` | 결과 그림, 단위 시험 |

## 3. 저장소 받기

```bash
git clone https://github.com/nhd2642/ros2_hw.git ~/ros2_hw
```

:::{warning}
**WSL 의 홈(`~`) 아래에** 받으세요. Windows 쪽(`/mnt/c/...`)에 받으면 `ws/src` 심볼릭 링크가 깨지고 빌드·실행이 크게 느려집니다.
:::

저장소 안의 `ws/src` 는 `code/ros2_hw_gz_src` 를 가리키는 링크입니다. 소스를 고칠 때는 어느 쪽 경로로 열어도 같은 파일입니다.

```bash
ls -l ~/ros2_hw/ws/src     # src -> ../code/ros2_hw_gz_src
```

## 4. 빌드

```bash
cd ~/ros2_hw/ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install      # 약 2 분
```

`--symlink-install` 을 쓰면 파이썬 노드와 launch 파일을 고친 뒤 다시 빌드하지 않아도 바로 반영됩니다 (새 파일을 추가했거나 C++ 를 고쳤을 때만 다시 빌드).

빌드가 끝나면 터미널마다 작업 공간을 불러옵니다.

```bash
source ~/ros2_hw/ws/install/setup.bash
```

:::{tip}
매번 치기 번거로우면 `~/.bashrc` 에 넣어 두세요.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_hw/ws/install/setup.bash" >> ~/.bashrc
```
:::

## 5. 사람 인식 모델 확인

MobileNet-SSD 모델 파일은 저장소에 들어 있습니다. 없으면 `survivor_detector` 가 경고만 내고 사람 인식을 건너뜁니다(탐색은 계속).

```bash
ls ~/ros2_hw/ws/person_ai_models/MobileNet-SSD/
# deploy.prototxt  mobilenet_iter_73000.caffemodel
```

## 6. 설치 확인

단위 시험은 ROS 를 띄우지 않고 로직만 시험하므로 1 분 안에 끝납니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_hw/ws/src/comm_relay && python3 -m pytest -q test
cd ~/ros2_hw/ws/src/terrain_perception && python3 -m pytest -q test
```

모두 통과하면 준비가 끝났습니다. 다음은 {doc}`quickstart` 입니다.

## 자주 막히는 곳

`colcon build` 에서 `ign_ros2_control` 을 못 찾음
: `ros-humble-ign-ros2-control` 이 설치되지 않았습니다. 2 단계 명령을 다시 실행하세요.

`ros2 launch` 에서 `package 'disaster_sim' not found`
: 이 터미널에서 `source ~/ros2_hw/ws/install/setup.bash` 를 안 했습니다.

빌드가 아주 느리거나 `ws/src` 가 비어 보임
: 저장소를 `/mnt/c` 아래에 받았을 가능성이 큽니다. WSL 홈으로 옮기세요.

그 밖의 문제는 {doc}`../how_to/troubleshooting` 을 보세요.
