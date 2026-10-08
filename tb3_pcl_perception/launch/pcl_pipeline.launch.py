#!/usr/bin/env python3
"""
D435i 포인트클라우드 3D 인지/시각화 파이프라인 (ROS 2 Jazzy).

체인 (pcl_ros 노드들을 코드 없이 런치로 연결 — 레퍼런스 pcl_processor.cpp 역할을 대체):
  카메라 pointcloud  (/camera/camera/depth/color/points)
     -> voxel_grid          다운샘플            -> /pcl/voxel
     -> statistical_outlier 노이즈 점 제거      -> /pcl/filtered
     -> (옵션) pointcloud_to_laserscan          -> /pcl/scan   (저상 장애물 2D 스캔)

용도: SLAM 지도 생성이 아니라 3D 장애물 인지/회피·시각화.
      (지도 생성은 tb3_lidar_vslam 담당 — 역할 분리)

전제: 이 런치가 카메라를 켤 수도(camera:=true), 이미 떠 있는 카메라를 쓸 수도(camera:=false) 있다.
      pointcloud 필터는 arm64 에서 이름이 'pointcloud__neon_.enable' 임.

사용:
  # 카메라까지 같이 (pointcloud 켜서)
  ros2 launch tb3_pcl_perception pcl_pipeline.launch.py camera:=true
  # 이미 카메라가 떠 있으면 (pointcloud 토픽만 소비)
  ros2 launch tb3_pcl_perception pcl_pipeline.launch.py camera:=false
  # 2D 가상 스캔까지
  ros2 launch tb3_pcl_perception pcl_pipeline.launch.py to_scan:=true

RViz: PointCloud2 /pcl/filtered (Best Effort) 로 처리된 클라우드 확인.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera  = LaunchConfiguration('camera')
    to_scan = LaunchConfiguration('to_scan')

    args = [
        DeclareLaunchArgument('camera', default_value='false',
                              description='true=D435i 카메라도 실행(pointcloud 켜서), false=기존 카메라 사용'),
        DeclareLaunchArgument('to_scan', default_value='false',
                              description='true=pointcloud_to_laserscan 으로 2D 가상 스캔(/pcl/scan) 생성'),
    ]

    pkg = FindPackageShare('tb3_pcl_perception')
    params = PathJoinSubstitution([pkg, 'config', 'pcl_params.yaml'])

    points_in = '/camera/camera/depth/color/points'

    # ---------- (옵션) D435i 카메라 — pointcloud 켜기 (arm64: NEON 접미사) ----------
    camera_node = Node(
        condition=IfCondition(camera),
        package='realsense2_camera', executable='realsense2_camera_node',
        namespace='camera', name='camera', output='screen',
        prefix='taskset -c 2,3',
        parameters=[{
            'pointcloud__neon_.enable': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'depth_module.depth_profile': '424x240x6',
            'rgb_camera.color_profile': '424x240x6',
        }],
    )

    # ---------- voxel 다운샘플 ----------
    voxel = Node(
        package='pcl_ros', executable='filter_voxel_grid_node',
        name='voxel_grid', output='screen',
        parameters=[params],
        remappings=[('input', points_in), ('output', '/pcl/voxel')],
    )

    # ---------- 통계적 outlier 제거 ----------
    outlier = Node(
        package='pcl_ros', executable='filter_statistical_outlier_removal_node',
        name='statistical_outlier_removal', output='screen',
        parameters=[params],
        remappings=[('input', '/pcl/voxel'), ('output', '/pcl/filtered')],
    )

    # ---------- (옵션) 3D cloud -> 2D 가상 스캔 ----------
    to_scan_node = Node(
        condition=IfCondition(to_scan),
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan', output='screen',
        parameters=[params],
        remappings=[('cloud_in', '/pcl/filtered'), ('scan', '/pcl/scan')],
    )

    return LaunchDescription(args + [camera_node, voxel, outlier, to_scan_node])
