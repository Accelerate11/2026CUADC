cmake_minimum_required(VERSION 3.8)
project(cuadc_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# The launch file imports profile_loader from the installed share directory.
install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Keep a standalone, installed preflight command for ground diagnostics.  The
# companion module is installed beside it so the command does not depend on a
# source-tree PYTHONPATH.
install(
  PROGRAMS scripts/preflight.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME preflight
)
install(
  FILES scripts/profile_loader.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
