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
find_package(rcl_interfaces REQUIRED)
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
  rcl_interfaces
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
  add_executable(test_stage_motion_profile test/test_stage_motion_profile.cpp)
  add_test(NAME stage_motion_profile COMMAND test_stage_motion_profile)
  add_executable(test_guided_profile_client test/test_guided_profile_client.cpp)
  add_test(NAME guided_profile_client COMMAND test_guided_profile_client)
  add_executable(test_guided_stage_integration test/test_guided_stage_integration.cpp)
  get_filename_component(CUADC_WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
  target_compile_definitions(test_guided_stage_integration PRIVATE CUADC_SOURCE_ROOT="${CUADC_WORKSPACE_ROOT}")
  add_executable(test_guided_profile_ros_adapter test/test_guided_profile_ros_adapter.cpp)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(test_guided_stage_integration PRIVATE -Wno-subobject-linkage)
  endif()
  ament_target_dependencies(test_guided_stage_integration
    geometry_msgs mavros_msgs nav_msgs sensor_msgs rclcpp rcl_interfaces std_msgs std_srvs)
  ament_target_dependencies(test_guided_profile_ros_adapter rclcpp mavros_msgs rcl_interfaces)
  add_test(NAME guided_stage_integration COMMAND test_guided_stage_integration)
  add_test(NAME guided_profile_ros_adapter COMMAND test_guided_profile_ros_adapter)
  set_tests_properties(guided_stage_integration guided_profile_ros_adapter PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=187;ROS_LOCALHOST_ONLY=1" TIMEOUT 30 LABELS "offline;regression")
  set_tests_properties(stage_motion_profile guided_profile_client PROPERTIES
    TIMEOUT 10 LABELS "offline;regression;standalone")
  add_executable(test_time_alignment test/test_time_alignment.cpp)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # Single-translation-unit test includes the implementation intentionally.
    target_compile_options(test_time_alignment PRIVATE -Wno-subobject-linkage)
  endif()
  ament_target_dependencies(test_time_alignment
    geometry_msgs mavros_msgs nav_msgs sensor_msgs rclcpp rcl_interfaces std_msgs std_srvs)
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
