# Installation

This page takes you from **a fresh Ubuntu 22.04 (including WSL2)** to a machine that can build the simulation. It takes about 20-30 minutes.

:::{note}
The project was developed and tested on **Windows 11 + WSL2 (Ubuntu 22.04)**. A regular Ubuntu 22.04 PC works with the same steps.
:::

## Requirements

| Item | Recommended |
| --- | --- |
| OS | Ubuntu 22.04 (Jammy), or WSL2 Ubuntu 22.04 on Windows 11 |
| CPU | 8+ cores. The simulation is CPU heavy (RTF around 0.3) |
| Memory | 16 GB+ |
| Disk | 15 GB+ free (ROS 2 + Gazebo + build) |
| GPU | Not needed. Sensors render with software GL |

## 1. Install ROS 2 Humble

If you don't have ROS 2 yet, add the repository as in the [official guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 2. Install the required packages

One command installs ROS 2 desktop, the Gazebo bridge (`ros_gz`), ros2_control, Nav2, slam_toolbox and OpenCV for person detection.

```bash
sudo apt update && sudo apt install -y ros-humble-desktop ros-humble-ros-gz ros-humble-ign-ros2-control \
  ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-robot-localization ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-broadcaster \
  python3-colcon-common-extensions python3-opencv python3-matplotlib python3-pytest
```

What each one is for:

| Package | Used for |
| --- | --- |
| `ros-humble-ros-gz`, `ros-humble-ign-ros2-control` | running Gazebo Sim (Fortress), topic bridge, A1 joint control |
| `ros-humble-ros2-control`, `ros-humble-ros2-controllers` | passing CHAMP joint commands to the simulated joints |
| `ros-humble-robot-localization` | A1 EKF (odom) |
| `ros-humble-slam-toolbox` | per-robot 2D SLAM |
| `ros-humble-nav2-bringup` | planning (NavFn) and following (MPPI, `nav2-mppi-controller` from `ros-humble-navigation2`) servers |
| `python3-opencv` | YOLOv4-tiny person detection (OpenCV DNN) |
| `python3-matplotlib`, `python3-pytest` | result plots, unit tests |

## 3. Get the repository

```bash
git clone https://github.com/nhd2642/ros2_hw.git ~/ros2_hw
```

:::{warning}
Clone it **under your WSL home (`~`)**. Under the Windows side (`/mnt/c/...`) the `ws/src` symlink breaks and builds and runs become very slow.
:::

`ws/src` inside the repository is a link to `code/ros2_hw_gz_src`. Either path opens the same files.

```bash
ls -l ~/ros2_hw/ws/src     # src -> ../code/ros2_hw_gz_src
```

## 4. Build

```bash
cd ~/ros2_hw/ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install      # about 2 minutes
```

With `--symlink-install`, edits to Python nodes and launch files take effect without rebuilding (rebuild only when you add files or change C++).

After the build, source the workspace in every terminal.

```bash
source ~/ros2_hw/ws/install/setup.bash
```

:::{tip}
If that gets tedious, add it to `~/.bashrc`.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_hw/ws/install/setup.bash" >> ~/.bashrc
```
:::

## 5. Check the person detection model

The YOLOv4-tiny model files are in the repository. Without them `survivor_detector` only logs a warning and skips detection (exploration continues).

```bash
ls ~/ros2_hw/ws/person_ai_models/yolov4-tiny/
# README.md  yolov4-tiny.cfg  yolov4-tiny.weights
```

If they are missing, download both files from the addresses in that folder's `README.md` (AlexeyAB/darknet public weights).

## 6. Verify the install

The unit tests check logic without starting ROS, so they finish in under a minute.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_hw/ws/src/comm_relay && python3 -m pytest -q test
cd ~/ros2_hw/ws/src/terrain_perception && python3 -m pytest -q test
```

If everything passes, you are ready. Next: {doc}`quickstart`.

## Common snags

`colcon build` cannot find `ign_ros2_control`
: `ros-humble-ign-ros2-control` is missing. Run step 2 again.

`package 'disaster_sim' not found` from `ros2 launch`
: This terminal has not run `source ~/ros2_hw/ws/install/setup.bash`.

Build is very slow or `ws/src` looks empty
: You most likely cloned under `/mnt/c`. Move it to your WSL home.

For anything else see {doc}`../how_to/troubleshooting`.
