cmake_minimum_required(VERSION 3.5)
project(robot_sim)

find_package(ament_cmake REQUIRED)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robot_sim_gazebo_model_path.sh.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/robot_sim_gazebo_model_path.dsv.in"
)

install(
  DIRECTORY
    launch
    worlds
    models
    urdf
    config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
