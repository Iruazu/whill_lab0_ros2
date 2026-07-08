cmake_minimum_required(VERSION 3.8)
project(whill_safety)

# Hybrid ament_cmake + ament_cmake_python: mirrors whill_sensors_bringup's
# pattern. M6R-2 ships launch files only; M6R-3 will add
# whill_safety/failsafe_node.py plus a config/twist_mux.yaml install
# block below. Keeping the CMake structure ready so the M6R-3 diff is
# small and additive.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# TODO(M6R-3): uncomment when failsafe_node lands
# install(
#   PROGRAMS
#     ${PROJECT_NAME}/failsafe_node.py
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME failsafe_node
# )

ament_package()
