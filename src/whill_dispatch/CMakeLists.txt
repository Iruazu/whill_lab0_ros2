cmake_minimum_required(VERSION 3.8)
project(whill_dispatch)

# Hybrid ament_cmake + ament_cmake_python, mirroring whill_safety. The M7
# plan (docs/ja/plans/2026-07-19-m7-dispatch.md) names ament_python, but
# every package in this repo is ament_cmake and this repo has no
# ament_python precedent; ament_cmake_python installs the pure-python
# dispatch_node identically (see whill_safety/failsafe_node) and the
# plan's stated reason for ament_python — "avoid the rosidl build" — is
# already satisfied here because we declare no custom interfaces. Staying
# on ament_cmake keeps the buildtool_depend rule (ament_cmake only)
# intact. Deviation logged in the S1 report.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# launch/  -> the rosbridge + dispatch_node + http.server composition
# web/     -> the static tablet UI (index.html, app.js, roslib.min.js, ...)
#             served verbatim by the http.server started in dispatch_launch.
install(
  DIRECTORY launch web
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    ${PROJECT_NAME}/dispatch_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dispatch_node
)

ament_package()
