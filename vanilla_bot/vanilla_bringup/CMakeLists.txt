cmake_minimum_required(VERSION 3.16)
project(vanilla_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES ../probe/robot_hardware.yaml ../probe/safety_czar.yaml
  DESTINATION share/${PROJECT_NAME}/contracts)
ament_package()
