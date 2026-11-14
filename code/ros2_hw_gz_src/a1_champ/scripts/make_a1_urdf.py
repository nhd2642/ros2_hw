#!/usr/bin/env python3
"""unitree_ros(chvmp fork)의 a1.urdf(ROS1용)를 ROS2 Humble + Gazebo Classic + CHAMP용 a1.urdf.xacro로 변환.
사용: python3 make_a1_urdf.py <a1.urdf> <출력 a1.urdf.xacro>
"""
import sys
import xml.etree.ElementTree as ET

JOINTS = [f"{leg}_{j}_joint" for leg in ("FL", "FR", "RL", "RR") for j in ("hip", "thigh", "calf")]

src, dst = sys.argv[1], sys.argv[2]
root = ET.parse(src).getroot()
root.set("name", "a1")

# 1) ROS1 transmission 전부 제거 (ROS2는 <ros2_control>로 대체)
for t in root.findall("transmission"):
    root.remove(t)

# 2) <gazebo> 안의 ROS1/unitree 플러그인, 센서, 빈 visual 제거
for g in root.findall("gazebo"):
    for parent in list(g.iter()):
        for child in list(parent):
            if child.tag in ("plugin", "sensor"):
                parent.remove(child)
    for v in g.findall("visual"):
        if len(v) == 0:
            g.remove(v)

# 3) 없는 링크(stick_link)를 가리키는 gazebo 블록, 내용이 빈 블록 제거
links = {l.get("name") for l in root.findall("link")}
for g in root.findall("gazebo"):
    ref = g.get("reference")
    if ref is not None and ref not in links and ref != "base":
        root.remove(g)
    elif len(g) == 0:
        root.remove(g)

# 4) base -> base_link (CHAMP 기본 규칙)
for el in root.iter():
    for attr in ("name", "link", "reference"):
        if el.get(attr) == "base":
            el.set(attr, "base_link")

# 5) 메시 경로 변경
for m in root.iter("mesh"):
    m.set("filename", m.get("filename").replace(
        "package://a1_description/", "file://$(find a1_champ)/"))

# 6) 검증: 관절 12개가 모두 revolute로 존재하는지
have = {j.get("name") for j in root.findall("joint") if j.get("type") == "revolute"}
missing = set(JOINTS) - have
if missing:
    sys.exit(f"관절 누락: {missing}")

body = ET.tostring(root, encoding="unicode")
body = body.replace('<robot name="a1">',
                    '<robot name="a1" xmlns:xacro="http://www.ros.org/wiki/xacro">', 1)

joint_ifaces = "\n".join(
    f"""    <joint name="{j}">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>""" for j in JOINTS)

extra = f"""
  <xacro:arg name="robot_ns" default="a1_1"/>
  <xacro:arg name="ros_control_yaml" default=""/>

  <!-- ===== ros2_control (CHAMP는 effort 명령을 사용) ===== -->
  <ros2_control name="a1_system" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
{joint_ifaces}
  </ros2_control>

  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <ros><namespace>/$(arg robot_ns)</namespace></ros>
      <robotNamespace>/$(arg robot_ns)</robotNamespace>
      <parameters>$(arg ros_control_yaml)</parameters>
    </plugin>
  </gazebo>

  <!-- ===== 정답 위치(odom/ground_truth), 디버깅용 ===== -->
  <gazebo>
    <plugin name="p3d_base_controller" filename="libgazebo_ros_p3d.so">
      <ros>
        <namespace>/$(arg robot_ns)</namespace>
        <remapping>odom:=odom/ground_truth</remapping>
      </ros>
      <body_name>base_link</body_name>
      <frame_name>world</frame_name>
      <update_rate>10.0</update_rate>
    </plugin>
  </gazebo>

  <!-- ===== IMU (CHAMP EKF가 imu/data를 사용) ===== -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <namespace>/$(arg robot_ns)</namespace>
          <remapping>~/out:=imu/data</remapping>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>

  <!-- ===== 2D LiDAR (SLAM용) ===== -->
  <link name="lidar_link">
    <inertial><mass value="0.05"/><inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/></inertial>
    <visual><geometry><cylinder radius="0.03" length="0.04"/></geometry></visual>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="trunk"/><child link="lidar_link"/>
    <origin xyz="0.0 0.0 0.09" rpy="0 0 0"/>
  </joint>
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">
      <always_on>true</always_on>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan><horizontal>
          <samples>360</samples><resolution>1</resolution>
          <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle>
        </horizontal></scan>
        <range><min>0.15</min><max>10.0</max><resolution>0.01</resolution></range>
      </ray>
      <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/$(arg robot_ns)</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

  <!-- ===== RGB 카메라 (마네킹 Vision 인식용) : /<ns>/front_camera/image_raw ===== -->
  <link name="camera_link">
    <inertial><mass value="0.02"/><inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/></inertial>
    <visual><geometry><box size="0.02 0.05 0.02"/></geometry></visual>
  </link>
  <joint name="camera_joint" type="fixed">
    <parent link="trunk"/><child link="camera_link"/>
    <origin xyz="0.14 0.0 0.03" rpy="0 0 0"/>
  </joint>
  <gazebo reference="camera_link">
    <sensor name="front_camera" type="camera">
      <always_on>true</always_on>
      <update_rate>15</update_rate>
      <camera>
        <horizontal_fov>1.8</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.1</near><far>15</far></clip>
      </camera>
      <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
        <ros><namespace>/$(arg robot_ns)</namespace></ros>
        <camera_name>front_camera</camera_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
"""
body = body.replace("</robot>", extra + "</robot>")
open(dst, "w", encoding="utf-8").write('<?xml version="1.0"?>\n' + body + "\n")
print("OK ->", dst)
