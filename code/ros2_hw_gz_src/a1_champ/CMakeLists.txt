cmake_minimum_required(VERSION 3.8)
project(a1_champ)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes config launch worlds
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gait_eval.py scripts/joint_tracking.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
