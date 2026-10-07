#!/usr/bin/env python3
"""
Visual-Inertial SLAM (RTAB-Map VIO) — D435i 카메라 + IMU 융합.

카메라만 쓰되 IMU를 융합(VIO)해 추적 정확도를 높인 vslam 구성.
  - imu_filter_madgwick : D435i raw IMU(/camera/camera/imu) -> 자세 포함 /imu/data
  - rgbd_odometry(VIO)  : RGB-D + IMU 로 시각-관성 오도메트리
  - rtabmap             : 그래프 SLAM + 루프클로저, 지도 생성(/rtabmap/*)

두 가지 모드:
  camera:=false (기본)  bag 재생용. 카메라는 안 띄우고 bag이 공급 (use_sim_time:=true 와 함께).
  camera:=true          라이브용. D435i(IMU 포함)를 직접 띄움 (코어 2,3 격리). use_sim_time:=false.

재생 예:
  # 터미널1
  ros2 launch ~/D435i/launch/vslam_vio.launch.py            # (camera:=false, use_sim_time:=true 기본)
  # 터미널2
  ros2 bag play ~/maps/run_XXXX --clock --rate 0.5
  # VMware RViz: Fixed Frame=map, Map=/rtabmap/map, PointCloud2=/rtabmap/cloud_map (use_sim_time:=true)

라이브 예:
  ros2 launch ~/D435i/launch/vslam_vio.launch.py camera:=true use_sim_time:=false

주의: bag 재생 모드면 그 bag에 /camera/camera/imu 가 녹화돼 있어야 함 (record_run.sh 가 포함함).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera = LaunchConfiguration('camera')
    use_sim_time = LaunchConfiguration('use_sim_time')
    frame_id = LaunchConfiguration('frame_id')

    args = [
        DeclareLaunchArgument('camera', default_value='false',
                              description='true=라이브(D435i 직접), false=bag 재생'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='bag 재생이면 true, 라이브면 false'),
        DeclareLaunchArgument('frame_id', default_value='base_footprint',
                              description='로봇 기준 프레임. bringup 없이 라이브 단독이면 camera_link'),
    ]

    rgb   = '/camera/camera/color/image_raw'
    depth = '/camera/camera/aligned_depth_to_color/image_raw'
    info  = '/camera/camera/color/camera_info'

    # ---- (라이브 전용) D435i + IMU, 코어 2,3 격리 ----
    camera_node = Node(
        condition=IfCondition(camera),
        package='realsense2_camera', executable='realsense2_camera_node',
        namespace='camera', name='camera', output='screen',
        prefix='taskset -c 2,3',
        parameters=[{
            'align_depth.enable': True,
            'depth_module.depth_profile': '424x240x6',
            'rgb_camera.color_profile': '424x240x6',
            'enable_infra1': False,  # IR 스트림 끔 (USB 대역폭 절약 — 과부하/연결끊김 방지)
            'enable_infra2': False,
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2,   # gyro+accel 를 /camera/camera/imu 로 통합
        }],
    )

    # ---- IMU 필터: raw IMU -> 자세(orientation) 포함 /imu/data ----
    imu_filter = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        name='imu_filter', output='screen',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),
            ('imu/data', '/imu/data'),
        ],
    )

    # ---- RTAB-Map VIO (rgbd_odometry 시각-관성 + rtabmap) ----
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'])),
        launch_arguments={
            'rgb_topic': rgb,
            'depth_topic': depth,
            'camera_info_topic': info,
            'imu_topic': '/imu/data',
            'wait_imu_to_init': 'true',
            'visual_odometry': 'true',      # rgbd_odometry 사용 (휠오돔 X, 순수 vslam)
            'frame_id': frame_id,
            'approx_sync': 'true',
            'qos': '2',                     # best_effort (bag/카메라 QoS 맞춤)
            'use_sim_time': use_sim_time,
            'rtabmap_viz': 'false',
            'rviz': 'false',
            'args': '-d',                   # 시작 시 이전 DB 삭제
        }.items(),
    )

    return LaunchDescription(args + [camera_node, imu_filter, rtabmap])
