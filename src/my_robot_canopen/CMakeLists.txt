cmake_minimum_required(VERSION 3.8)
project(my_robot_canopen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(canopen_402_driver REQUIRED)
find_package(canopen_master_driver REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(lely_core_libraries REQUIRED)

generate_dcf(robot_bus)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

