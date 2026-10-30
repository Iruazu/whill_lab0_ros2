cmake_minimum_required(VERSION 3.8)
project(whill_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Collision Monitor polygons vs the costmap footprint; yaml only, no graph.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_collision_monitor_config
    test/test_collision_monitor_config.py)
endif()

ament_package()
