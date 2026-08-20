cmake_minimum_required(VERSION 3.8)
project(cuadc_mission)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(cuadc_interfaces REQUIRED)
find_package(cuadc_recon REQUIRED)
find_package(cuadc_vehicle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mission_node src/mission_node.cpp)
target_compile_features(mission_node PUBLIC cxx_std_17)
ament_target_dependencies(
  mission_node
  cuadc_interfaces
  cuadc_recon
  cuadc_vehicle
  geometry_msgs
  mavros_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
)

install(TARGETS mission_node DESTINATION lib/${PROJECT_NAME})

ament_package()
