cmake_minimum_required(VERSION 3.8)
project(my_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/three_motor_position_cli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME three_motor_position_cli
)

install(
  DIRECTORY config urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
