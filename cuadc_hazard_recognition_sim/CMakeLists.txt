cmake_minimum_required(VERSION 3.8)
project(cuadc_hazard_recognition_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  docs
  launch
  models
  scripts
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
