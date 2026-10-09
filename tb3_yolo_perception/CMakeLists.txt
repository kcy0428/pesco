cmake_minimum_required(VERSION 3.8)
project(tb3_yolo_perception)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/yolo_detect.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
