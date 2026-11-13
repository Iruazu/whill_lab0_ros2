cmake_minimum_required(VERSION 3.8)
project(whill_sensors_bringup)

find_package(ament_cmake REQUIRED)
# ament_cmake_python は ament_python_install_package で Python モジュールを
# インストールするためだけに足している。本パッケージは launch 中心で
# Python ノードは imu_sign_corrector の 1 本だけなので、パッケージ全体を
# ament_python に切り替えるほどではない (ROS 2 公式が許容するハイブリッド
# パターン: ament_cmake + ament_python_install_package)。
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Python entry point. ament_python_install_package が
# lib/python3.x/site-packages 以下にモジュールを置くので、ros2 run から
# 呼べるよう lib/${PROJECT_NAME}/ にも実行可能スクリプトとしてインストール
# する。setup.py の console_scripts 相当のことを CMake 側でやっている形。
install(
  PROGRAMS
    ${PROJECT_NAME}/imu_sign_corrector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_sign_corrector
)

if(BUILD_TESTING)
  # 終了処理の回帰テスト (Issue #169)。ノードを別プロセスで起動してシグナルを
  # 送るだけで、センサも launch も要らない。
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_imu_sign_corrector_shutdown
    test/test_imu_sign_corrector_shutdown.py
    TIMEOUT 120)
endif()

ament_package()
