cmake_minimum_required(VERSION 3.8)
project(tb3_lidar_vslam)

find_package(ament_cmake REQUIRED)

# 런치 / 설정 / 스크립트 설치
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/record_run.sh
  scripts/scan_view.py
  scripts/scan_fixer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
