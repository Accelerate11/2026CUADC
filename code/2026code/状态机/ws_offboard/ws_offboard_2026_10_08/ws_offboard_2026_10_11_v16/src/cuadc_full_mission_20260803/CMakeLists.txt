cmake_minimum_required(VERSION 3.8)
project(cuadc_full_mission_20260803)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wconversion
    -Wshadow
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(
  cuadc_full_mission_node
  src/cuadc_full_mission_node.cpp
)

ament_target_dependencies(
  cuadc_full_mission_node
  geometry_msgs
  mavros_msgs
  nav_msgs
  sensor_msgs
  rclcpp
  std_msgs
  std_srvs
)

install(
  TARGETS cuadc_full_mission_node
  DESTINATION lib/${PROJECT_NAME}
)

# 实机视觉节点和它依赖的分割/深度分析模块必须安装到同一目录
install(
  PROGRAMS
    scripts/basket_vision_ros_node.py
    scripts/capture_clock.py
    scripts/basket_detect_seg_analysis.py
    scripts/mavlink_stream_configurator.py
    scripts/flight_telemetry_recorder.py
    scripts/preflight_camera_view.py
  DESTINATION lib/${PROJECT_NAME}
)

# 只安装实机参数、实机启动文件、模型和检查文档
install(
  DIRECTORY
    config
    docs
    launch
    models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  enable_testing()
  # Pure V16 gate regression tests can also be built without ROS:
  # cmake -S test -B build-release-gate && cmake --build build-release-gate
  add_subdirectory(test)
  add_executable(test_time_alignment test/test_time_alignment.cpp)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # Single-translation-unit test includes the implementation intentionally.
    target_compile_options(test_time_alignment PRIVATE -Wno-subobject-linkage)
  endif()
  ament_target_dependencies(test_time_alignment
    geometry_msgs mavros_msgs nav_msgs sensor_msgs rclcpp std_msgs std_srvs)
  add_test(NAME time_alignment COMMAND test_time_alignment)
  set_tests_properties(time_alignment PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=187;ROS_LOCALHOST_ONLY=1")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME capture_clock COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_capture_clock.py)
  add_test(NAME vision_backend COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_vision_backend.py)
  set_tests_properties(time_alignment capture_clock vision_backend PROPERTIES
    TIMEOUT 30
    LABELS "offline;regression")
  set_tests_properties(capture_clock vision_backend PROPERTIES
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
endif()

ament_package()
