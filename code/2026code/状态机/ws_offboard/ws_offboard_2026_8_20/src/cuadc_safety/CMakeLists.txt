cmake_minimum_required(VERSION 3.8)
project(cuadc_safety)

find_package(ament_cmake REQUIRED)
find_package(cuadc_interfaces REQUIRED)
find_package(cuadc_recon REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(safety_monitor src/safety_monitor.cpp)
target_compile_features(safety_monitor PRIVATE cxx_std_17)
target_compile_options(safety_monitor PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
ament_target_dependencies(safety_monitor
  cuadc_interfaces geometry_msgs mavros_msgs nav_msgs rclcpp)
target_link_libraries(safety_monitor cuadc_recon::cuadc_recon_evidence)

install(TARGETS safety_monitor DESTINATION lib/${PROJECT_NAME})
ament_package()
