cmake_minimum_required(VERSION 3.8)
project(cuadc_rescue_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(virtual_drop_judge_node src/virtual_drop_judge_node.cpp)
ament_target_dependencies(
  virtual_drop_judge_node
  rclcpp
  nav_msgs
  std_srvs
)

add_executable(simple_mission_demo_node src/simple_mission_demo_node.cpp)
ament_target_dependencies(
  simple_mission_demo_node
  rclcpp
  geometry_msgs
  std_srvs
)

install(TARGETS
  virtual_drop_judge_node
  simple_mission_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  models
  scripts
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
