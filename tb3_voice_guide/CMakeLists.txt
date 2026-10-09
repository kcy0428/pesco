cmake_minimum_required(VERSION 3.8)
project(tb3_voice_guide)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/voice_guide.py
  scripts/save_waypoint.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
