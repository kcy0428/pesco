#!/usr/bin/env python3
"""
병렬 SLAM 비교 런치: LiDAR SLAM(slam_toolbox) + Visual SLAM(RTAB-Map) 동시 실행.

구조 (TF 충돌 방지가 핵심):
  - slam_toolbox : 실제 LDS 라이다 /scan 사용. TF(map->odom)를 '소유'. 맵 = /map. (RViz로 봄)
  - RTAB-Map     : D435i RGB-D만 사용(라이다 X). publish_tf:=false 로 TF를 '발행하지 않고'
                   /odom + TF를 듣기만 함. 맵 = /rtabmap/*. (rtabmap_viz 로 봄)
  둘 다 휠 /odom 을 공통 기준으로 사용 → 변수 통제.

전제: turtlebot3_bringup robot.launch.py 가 이미 실행 중 (라이다 /scan, /odom, base TF 공급).

사용:
  source /opt/ros/jazzy/setup.bash && source ~/D435i/install/setup.bash
  export ROS_DOMAIN_ID=30
  ros2 launch ~/D435i/launch/dual_slam.launch.py

인자:
  camera:=false       D435i 카메라 노드 끔(이미 따로 띄운 경우)
  lidar_slam:=false   slam_toolbox 끔
  visual_slam:=false  RTAB-Map 끔
  visual_odom:=true   RTAB-Map을 휠오돔 대신 시각오도메트리(rgbd_odometry)로 (무거움, 완전 카메라기반)
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera      = LaunchConfiguration('camera')
    lidar_slam  = LaunchConfiguration('lidar_slam')
    visual_slam = LaunchConfiguration('visual_slam')
    visual_odom = LaunchConfiguration('visual_odom')
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('lidar_slam', default_value='true'),
        DeclareLaunchArgument('visual_slam', default_value='true'),
        DeclareLaunchArgument('visual_odom', default_value='false',
                              description='RTAB-Map 오도메트리: false=휠/odom(권장), true=시각오도메트리'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    # 카메라 입력 토픽 (RTAB-Map은 color에 정합된 aligned depth 사용)
    rgb_topic   = '/camera/camera/color/image_raw'
    depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
    info_topic  = '/camera/camera/color/camera_info'

    # ---- D435i 카메라 (align_depth 켬: RTAB-Map용 정합 깊이) ----
    camera_node = Node(
        condition=IfCondition(camera),
        package='realsense2_camera', executable='realsense2_camera_node',
        namespace='camera', name='camera', output='screen',
        parameters=[{
            'align_depth.enable': True,
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
            'enable_gyro': False,
            'enable_accel': False,
            'pointcloud__neon_.enable': False,  # RTAB-Map이 자체 클라우드 생성 → CPU 절약
        }],
    )

    # ---- LiDAR SLAM: slam_toolbox (실제 라이다 /scan, TF 소유) ----
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(lidar_slam),
    )

    # ---- Visual SLAM: RTAB-Map (카메라만, publish_tf=false, 네임스페이스 rtabmap) ----
    rtabmap_common = {
        'frame_id': 'base_footprint',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': False,          # 라이다 안 씀 (순수 vslam)
        'approx_sync': True,
        'use_sim_time': use_sim_time,
        'publish_tf': False,              # ★ map->odom TF 발행 안 함 (slam_toolbox와 충돌 방지)
        'Mem/IncrementalMemory': 'true',
        'Reg/Force3DoF': 'true',          # 지상 로봇: 2D 평면 가정
    }
    rtabmap_remaps = [
        ('rgb/image', rgb_topic),
        ('depth/image', depth_topic),
        ('rgb/camera_info', info_topic),
        ('odom', '/odom'),
    ]

    # 휠 오도메트리 사용 시: odom_frame_id 로 /odom 사용. 시각오도메트리 사용 시 rgbd_odometry 추가.
    rtabmap_wheelodom = Node(
        condition=IfCondition(visual_slam),
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        namespace='rtabmap', output='screen',
        parameters=[dict(rtabmap_common, **{'odom_frame_id': 'odom'})],
        remappings=rtabmap_remaps,
        arguments=['-d'],                 # 시작 시 이전 DB 삭제
    )

    # 시각 오도메트리 경로 (visual_odom:=true 일 때): rgbd_odometry + rtabmap(odom 토픽 구독)
    vo_node = Node(
        condition=IfCondition(visual_odom),
        package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry',
        namespace='rtabmap', output='screen',
        parameters=[{'frame_id': 'base_footprint', 'approx_sync': True,
                     'publish_tf': False, 'use_sim_time': use_sim_time}],
        remappings=[('rgb/image', rgb_topic), ('depth/image', depth_topic),
                    ('rgb/camera_info', info_topic), ('odom', '/rtabmap/vo')],
    )
    rtabmap_visodom = Node(
        condition=IfCondition(visual_odom),
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        namespace='rtabmap', output='screen',
        parameters=[rtabmap_common],
        remappings=rtabmap_remaps[:-1] + [('odom', '/rtabmap/vo')],
        arguments=['-d'],
    )

    visual = GroupAction(condition=IfCondition(visual_slam), actions=[
        # 휠오돔 경로는 visual_odom=false 일 때만
        GroupAction(condition=UnlessCondition(visual_odom), actions=[rtabmap_wheelodom]),
        vo_node,
        rtabmap_visodom,
    ])

    return LaunchDescription(args + [camera_node, lidar, visual])
