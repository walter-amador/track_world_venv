cmake_minimum_required(VERSION 3.5)
project(robot_sim)

find_package(ament_cmake REQUIRED)

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
