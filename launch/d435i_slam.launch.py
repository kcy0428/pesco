#!/usr/bin/env python3
"""
D435i SLAM 통합 런치 (voice-guided guide robot 프로젝트)

한 줄로 다음을 모두 실행한다:
  1) RealSense D435i 카메라 노드  (pointcloud 켜진 채, 저해상도)
  2) depthimage_to_laserscan      (depth -> /scan 변환)
  3) slam_toolbox (online async)  (/scan + /odom + TF 로 지도 작성)

전제: TurtleBot3 bringup(robot.launch.py)이 이미 실행 중이어야 한다.
      (그래야 /odom 과 base_footprint/base_link/camera_* TF 가 들어온다)

사용:
  source /opt/ros/jazzy/setup.bash
  source ~/D435i/install/setup.bash
  export ROS_DOMAIN_ID=30
  ros2 launch ~/D435i/launch/d435i_slam.launch.py

개별 끄기 예:
  ros2 launch ~/D435i/launch/d435i_slam.launch.py slam:=false      # 매핑 빼고 카메라+scan만
  ros2 launch ~/D435i/launch/d435i_slam.launch.py scan:=false slam:=false   # 카메라만
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- 런치 인자 ----
    args = [
        DeclareLaunchArgument('camera', default_value='true',
                              description='D435i 카메라 노드 실행'),
        DeclareLaunchArgument('scan', default_value='true',
                              description='depthimage_to_laserscan 실행 (/scan 생성)'),
        DeclareLaunchArgument('slam', default_value='true',
                              description='slam_toolbox 실행 (지도 작성)'),
        DeclareLaunchArgument('depth_profile', default_value='640x480x15',
                              description='깊이 스트림 WxHxFPS'),
        DeclareLaunchArgument('color_profile', default_value='640x480x15',
                              description='컬러 스트림 WxHxFPS'),
        DeclareLaunchArgument('scan_output_frame', default_value='camera_depth_frame',
                              description='/scan 의 frame_id. RViz에서 방향 틀리면 '
                                          'camera_depth_optical_frame 로 변경'),
        DeclareLaunchArgument('range_min', default_value='0.3'),
        DeclareLaunchArgument('range_max', default_value='4.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- 1) RealSense D435i 카메라 ----
    # 컴포넌트 실행파일을 직접 Node 로 띄워 파라미터를 확실하게 주입한다.
    # arm64(NEON)에서는 pointcloud 필터 파라미터 이름이 'pointcloud__neon_.enable' 이다.
    camera_node = Node(
        condition=IfCondition(LaunchConfiguration('camera')),
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        parameters=[{
            'pointcloud__neon_.enable': True,
            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2,
        }],
    )

    # ---- 2) depth -> /scan 변환 ----
    scan_node = Node(
        condition=IfCondition(LaunchConfiguration('scan')),
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'output_frame': LaunchConfiguration('scan_output_frame'),
            'scan_height': 10,
            'range_min': LaunchConfiguration('range_min'),
            'range_max': LaunchConfiguration('range_max'),
        }],
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/camera/depth/camera_info'),
            ('scan', '/scan'),
        ],
    )

    # ---- 3) slam_toolbox (online async) ----
    slam = GroupAction(
        condition=IfCondition(LaunchConfiguration('slam')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch', 'online_async_launch.py',
                ])),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            ),
        ],
    )

    return LaunchDescription(args + [camera_node, scan_node, slam])
