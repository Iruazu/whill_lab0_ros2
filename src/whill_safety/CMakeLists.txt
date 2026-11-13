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

install(
  PROGRAMS
    ${PROJECT_NAME}/failsafe_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME failsafe_node
)

if(BUILD_TESTING)
  # failsafe_node の判定（救援・未着入力）を、時刻を差し替えて確かめる。
  # rclpy の context は要るが、ROS グラフの通信も待ち時間も要らない。
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_failsafe_rescue test/test_failsafe_rescue.py)
  ament_add_pytest_test(test_failsafe_never_arrived
    test/test_failsafe_never_arrived.py)
endif()

ament_package()
