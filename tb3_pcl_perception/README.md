# tb3_pcl_perception

D435i 포인트클라우드를 이용한 **3D 장애물 인지/시각화** 파이프라인 (ROS 2 Jazzy).

`turtlebot3-autonomy-stack` 레퍼런스의 Stage 3 PCL 파이프라인(voxel → outlier → ...)을,
C++ 노드 작성 없이 **`pcl_ros` 기본 필터 노드들을 런치로 체이닝**해 Pi에서 가볍게 구현한다.

> 역할 분리: **SLAM 지도 생성은 `tb3_lidar_vslam`** 이 담당한다. 이 패키지는 지도가 아니라
> **3D 장애물 인지/회피·시각화** 용도다. vSLAM 입력은 RGB-D 가 정석이므로(포인트클라우드를
> SLAM 입력으로 주면 RGB 특징을 잃어 품질↓), 포인트클라우드는 여기서 인지용으로만 쓴다.

## 파이프라인
```
/camera/camera/depth/color/points
  → voxel_grid (다운샘플)                → /pcl/voxel
  → statistical_outlier_removal (노이즈)  → /pcl/filtered
  → (옵션) pointcloud_to_laserscan        → /pcl/scan   (저상 장애물 2D 스캔)
```

## 사용법
```bash
# 카메라까지 같이 (pointcloud 켜서)
ros2 launch tb3_pcl_perception pcl_pipeline.launch.py camera:=true
# 이미 카메라가 떠 있으면
ros2 launch tb3_pcl_perception pcl_pipeline.launch.py camera:=false
# 2D 가상 스캔까지 (pointcloud_to_laserscan 설치 필요)
ros2 launch tb3_pcl_perception pcl_pipeline.launch.py to_scan:=true
```
RViz: `PointCloud2` → `/pcl/filtered` (Reliability = Best Effort).

## 파일
- `launch/pcl_pipeline.launch.py` — voxel → outlier (→ to_laserscan) 체인
- `config/pcl_params.yaml` — voxel leaf size, outlier mean_k/stddev, scan 변환 범위

## 의존성 메모
- `pcl_ros` : 설치됨 (filter_voxel_grid_node, filter_statistical_outlier_removal_node)
- `pointcloud_to_laserscan` : **미설치** — `to_scan:=true` 쓰려면
  `sudo apt install ros-jazzy-pointcloud-to-laserscan`
- arm64 에서 카메라 pointcloud 파라미터는 **`pointcloud__neon_.enable`** (NEON 접미사)

## Nav2 연동 (향후)
`/pcl/filtered` 를 Nav2 로컬 코스트맵의 **voxel layer** observation source 로 넣으면
라이다가 못 보는 저상/공중 3D 장애물을 회피에 반영할 수 있다 (레퍼런스의 3중 센서 코스트맵).
