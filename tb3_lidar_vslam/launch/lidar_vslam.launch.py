#!/usr/bin/env python3
"""
LiDAR 2D SLAM + D435i Visual(-Inertial) SLAM 통합 런치 — 비교(side-by-side) 구성.

설계 (turtlebot3-autonomy-stack 레퍼런스 철학 + 우리 2-Pi 환경):
  - LiDAR SLAM (slam_toolbox) : 실제 LDS /scan 사용. map->odom TF를 '소유'. 지도 = /map.
  - Visual SLAM (RTAB-Map)    : D435i RGB-D(+IMU) 사용. publish_tf:=false 로 TF 발행 안 함
                                (slam_toolbox와 single map->odom 충돌 방지). 지도 = /rtabmap/*.
  두 SLAM은 서로 융합하지 않고 '따로' 돌며, 같은 주행에서 두 지도를 비교한다.
  (센서 융합이 필요하면 Nav2 코스트맵 레벨에서 /scan + 카메라 포인트클라우드를 관측원으로 합침)

2-Pi 분산 사용 (동일 ROS_DOMAIN_ID + 시계동기화 chrony 전제):
  로봇 Pi   : ros2 launch tb3_lidar_vslam lidar_vslam.launch.py camera:=false vslam:=false lidar:=true
  카메라 Pi : ros2 launch tb3_lidar_vslam lidar_vslam.launch.py camera:=true vslam:=true lidar:=false \
                                                                frame_id:=base_footprint
  (로봇 Pi 에서 turtlebot3_bringup robot.launch.py 는 별도로 먼저 실행)

1-Pi / bag 재생:
  ros2 launch tb3_lidar_vslam lidar_vslam.launch.py use_sim_time:=true   (+ ros2 bag play ... --clock)

주요 인자:
  lidar:=true|false        slam_toolbox (라이다 2D SLAM) 실행
  camera:=true|false       D435i 카메라 노드 실행 (IR 끔, align_depth 켬, IMU 켬)
  vslam:=true|false        RTAB-Map (카메라 vslam, publish_tf=false) 실행
  use_vio:=true|false      RTAB-Map 오도메트리: true=시각관성(rgbd_odometry+IMU), false=휠 /odom
  frame_id:=...            vslam 기준 프레임. bringup 함께면 base_footprint, 카메라 단독이면 camera_link
  use_sim_time:=true|false bag 재생이면 true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar        = LaunchConfiguration('lidar')
    camera       = LaunchConfiguration('camera')
    vslam        = LaunchConfiguration('vslam')
    use_vio      = LaunchConfiguration('use_vio')
    frame_id     = LaunchConfiguration('frame_id')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('lidar', default_value='true',
                              description='slam_toolbox (LiDAR 2D SLAM, owns map->odom TF)'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='D435i 카메라 노드 (IR off, align_depth on, IMU on)'),
        DeclareLaunchArgument('vslam', default_value='true',
                              description='RTAB-Map (camera vslam, publish_tf=false)'),
        DeclareLaunchArgument('use_vio', default_value='true',
                              description='true=visual-inertial(rgbd_odometry+IMU), false=wheel /odom'),
        DeclareLaunchArgument('frame_id', default_value='base_footprint',
                              description='vslam 기준 프레임. 카메라 단독(bringup 없음)이면 camera_link'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='bag 재생이면 true'),
    ]

    pkg = FindPackageShare('tb3_lidar_vslam')
    rgb   = '/camera/camera/color/image_raw'
    depth = '/camera/camera/aligned_depth_to_color/image_raw'
    info  = '/camera/camera/color/camera_info'

    # ---------- scan_fixer: /scan 점 개수 보정 -> /scan_fixed (LDS-02 + slam_toolbox 궁합) ----------
    scan_fixer = Node(
        condition=IfCondition(lidar),
        package='tb3_lidar_vslam', executable='scan_fixer.py',
        name='scan_fixer', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---------- LiDAR SLAM: slam_toolbox (TF 소유, /scan_fixed 구독) ----------
    lidar_slam = Node(
        condition=IfCondition(lidar),
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[
            PathJoinSubstitution([pkg, 'config', 'slam_toolbox.yaml']),
            {'use_sim_time': use_sim_time},
        ],
    )

    # ---------- D435i 카메라 (IR off = USB 안정, align_depth on, IMU on) ----------
    camera_node = Node(
        condition=IfCondition(camera),
        package='realsense2_camera', executable='realsense2_camera_node',
        namespace='camera', name='camera', output='screen',
        prefix='taskset -c 2,3',    # 모터/라이다(코어 0,1) 안 굶게 카메라를 코어 2,3에 격리
        parameters=[{
            'align_depth.enable': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'depth_module.depth_profile': '424x240x6',
            'rgb_camera.color_profile': '424x240x6',
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2,
        }],
    )

    # ---------- IMU 필터 (VIO용): raw IMU -> /imu/data (자세 포함) ----------
    imu_filter = Node(
        condition=IfCondition(vslam),
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        name='imu_filter', output='screen',
        parameters=[{'use_mag': False, 'world_frame': 'enu',
                     'publish_tf': False, 'use_sim_time': use_sim_time}],
        remappings=[('imu/data_raw', '/camera/camera/imu'), ('imu/data', '/imu/data')],
    )

    # ---------- Visual SLAM: RTAB-Map (publish_tf=false → slam_toolbox와 충돌 방지) ----------
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'])),
        condition=IfCondition(vslam),
        launch_arguments={
            'rgb_topic': rgb,
            'depth_topic': depth,
            'camera_info_topic': info,
            'imu_topic': '/imu/data',
            'wait_imu_to_init': use_vio,
            'visual_odometry': use_vio,       # true=rgbd_odometry(VIO), false=휠 odom 토픽 사용
            'odom_topic': '/odom',            # use_vio=false 일 때 휠 오도메트리
            'frame_id': frame_id,
            'approx_sync': 'true',
            'qos': '2',                       # best_effort (카메라/ bag QoS)
            'publish_tf': 'false',            # ★ map->odom TF 발행 안 함
            'rtabmap_viz': 'false',
            'rviz': 'false',
            'args': '-d --Reg/Force3DoF true',  # 이전 DB 삭제 + 지상 로봇 2D 가정
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription(args + [scan_fixer, lidar_slam, camera_node, imu_filter, rtabmap])
