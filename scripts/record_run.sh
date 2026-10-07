#!/usr/bin/env bash
# rosbag 녹화 — LiDAR SLAM vs Visual SLAM 비교용 센서 데이터 수집.
# SLAM은 돌리지 않고(= CPU 여유) 센서/TF/오도메트리만 녹화한다.
# 나중에 이 bag을 재생(ros2 bag play --clock)하며 slam_toolbox / RTAB-Map 을 각각 돌려 비교.
#
# 전제: turtlebot3_bringup(라이다 /scan, /odom, /tf) + D435i 카메라(align_depth) 가 실행 중.
# 사용:  ~/D435i/scripts/record_run.sh [출력이름]
#        (기본 출력: ~/maps/run_<날짜시각>)

set -e
OUT="${1:-$HOME/maps/run_$(date +%Y%m%d_%H%M%S)}"

exec ros2 bag record -o "$OUT" \
  /scan \
  /odom \
  /tf \
  /tf_static \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/aligned_depth_to_color/camera_info \
  /camera/camera/imu
