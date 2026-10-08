# tb3_lidar_vslam

TurtleBot3 Waffle + Intel RealSense D435i 에서 **LiDAR 2D SLAM** 과 **카메라 Visual(-Inertial) SLAM**
을 나란히 돌려 비교하는 ROS 2 Jazzy 패키지.

`turtlebot3-autonomy-stack`(saman-aboutorab) 레퍼런스의 설계 철학을 따랐다: 라이다 2D SLAM이
주 지도를 만들고, D435i는 RGB-D SLAM 을 **별도로** 돌려 비교한다. 두 SLAM 은 하나로 **융합하지 않는다**
(센서 융합이 필요하면 지도 단계가 아니라 Nav2 코스트맵의 관측원 레벨에서 합친다).

## 구조

| | LiDAR SLAM | Visual SLAM |
|---|---|---|
| 노드 | slam_toolbox | RTAB-Map (+ imu_filter_madgwick) |
| 센서 | 실제 LDS `/scan` | D435i RGB-D (+ IMU) |
| map→odom TF | **소유** | 발행 안 함 (`publish_tf:=false`) |
| 지도 | `/map` | `/rtabmap/map`, `/rtabmap/cloud_map` |

핵심: `map→odom` TF 는 트리에 하나만 존재할 수 있으므로 slam_toolbox 가 소유하고 RTAB-Map 은
발행하지 않는다. 그래서 두 SLAM 이 충돌 없이 동시에 돈다.

## 전제 조건
- `turtlebot3_bringup robot.launch.py` 가 먼저 실행되어 `/scan`(실제 라이다), `/odom`, base TF 공급
- 2-Pi 분산이면: 두 Pi 가 동일 `ROS_DOMAIN_ID` + **chrony 시계동기화**, 가능하면 유선 연결

## 사용법

### 2-Pi 분산 (권장 — Pi5 과부하 회피)
```bash
# 로봇 Pi (Waffle): bringup 먼저, 그다음 라이다 SLAM만
ros2 launch turtlebot3_bringup robot.launch.py
ros2 launch tb3_lidar_vslam lidar_vslam.launch.py camera:=false vslam:=false lidar:=true

# 카메라 Pi: 카메라 + vslam만
ros2 launch tb3_lidar_vslam lidar_vslam.launch.py lidar:=false camera:=true vslam:=true
```

### 1-Pi bag 재생 비교
```bash
# 녹화 (SLAM 미실행)
ros2 run tb3_lidar_vslam record_run.sh
# 재생 + 라이다 SLAM
ros2 launch tb3_lidar_vslam lidar_vslam.launch.py camera:=false vslam:=false use_sim_time:=true
ros2 bag play ~/maps/run_XXXX --clock
# (같은 bag 으로 vslam 만 다시 재생해 두 지도 비교)
```

### 지도 저장
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/lidar_map          # 라이다 2D 지도
# RTAB-Map 은 ~/.ros/rtabmap.db 에 자동 저장; 격자지도는:
ros2 run nav2_map_server map_saver_cli -f ~/maps/vslam_map --ros-args -r map:=/rtabmap/map
```

## 주요 인자
- `lidar` / `camera` / `vslam` : 각 부분 on/off (2-Pi 분산 시 역할 분리용)
- `use_vio` : true=시각관성 오도메트리(rgbd_odometry+IMU), false=휠 `/odom`
- `frame_id` : vslam 기준 프레임 (bringup 함께면 `base_footprint`, 카메라 단독이면 `camera_link`)
- `use_sim_time` : bag 재생이면 true

## 파일
- `launch/lidar_vslam.launch.py` — 통합 런치
- `config/slam_toolbox.yaml` — 라이다 SLAM 파라미터
- `scripts/record_run.sh` — 비교용 센서 rosbag 녹화

## 알려진 이슈 / 팁
- D435i 는 IR+depth+color 동시 스트림 시 USB 에서 떨어짐 → **IR 비활성화**(런치에 반영됨)
- 카메라를 코어 2,3 에 격리(`taskset`)해 모터 시리얼(코어 0,1) 을 굶기지 않음
- vslam 추적은 밝고 특징 많은 장면에서 유지됨 (민무늬 벽/저조도에서 품질↓)
