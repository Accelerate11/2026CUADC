cmake_minimum_required(VERSION 3.8)
project(cuadc_hazard_recognition_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HazardDetection.msg"
  DEPENDENCIES std_msgs
)

add_executable(recon_state_machine_cpp src/recon_state_machine_cpp.cpp)
target_include_directories(recon_state_machine_cpp PRIVATE ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(recon_state_machine_cpp ${YAML_CPP_LIBRARIES})
ament_target_dependencies(recon_state_machine_cpp rclcpp geometry_msgs nav_msgs mavros_msgs sensor_msgs std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(recon_state_machine_cpp "${cpp_typesupport_target}")

install(TARGETS recon_state_machine_cpp DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/hazard_vision_node.py
  scripts/gz_image_bridge_node.py
  scripts/static_camera_roi_node.py
  scripts/evaluate_coverage_result.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config docs launch models scripts worlds DESTINATION share/${PROJECT_NAME})

ament_package()
