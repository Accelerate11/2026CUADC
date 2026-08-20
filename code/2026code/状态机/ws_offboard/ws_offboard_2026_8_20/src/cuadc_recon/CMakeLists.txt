cmake_minimum_required(VERSION 3.8)
project(cuadc_recon)

find_package(ament_cmake REQUIRED)
find_package(cuadc_core REQUIRED)
find_package(cuadc_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} src/route_planner.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(${PROJECT_NAME} cuadc_core)

add_library(${PROJECT_NAME}_evidence src/evidence_recorder.cpp)
target_compile_features(${PROJECT_NAME}_evidence PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_evidence PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_include_directories(${PROJECT_NAME}_evidence PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_evidence
  cuadc_interfaces cv_bridge rclcpp sensor_msgs)
target_link_libraries(${PROJECT_NAME}_evidence ${OpenCV_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_evidence EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  cuadc_core cuadc_interfaces cv_bridge OpenCV rclcpp sensor_msgs)
ament_export_include_directories(include)
ament_package()
