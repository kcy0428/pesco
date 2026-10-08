#!/usr/bin/env bash
# rosbag 녹화 — LiDAR SLAM vs Visual SLAM 비교용 센서 수집 (SLAM 미실행 = CPU 여유).
# 재생 시 slam_toolbox / RTAB-Map 를 각각 돌려 같은 주행으로 두 지도를 비교한다.
#
# 전제: turtlebot3_bringup(/scan,/odom,/tf) + D435i 카메라(align_depth, IMU) 실행 중.
# 사용:  record_run.sh [출력이름]     (기본: ~/maps/run_<날짜시각>)

set -e
OUT="${1:-$HOME/maps/run_$(date +%Y%m%d_%H%M%S)}"

exec ros2 bag record -o "$OUT" \
  /scan /odom /tf /tf_static \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/aligned_depth_to_color/camera_info \
  /camera/camera/imu
