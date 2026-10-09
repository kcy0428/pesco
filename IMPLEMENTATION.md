# 구현 정리 — D435i vSLAM + YOLO 객체인식 (TurtleBot3 / ROS 2 Jazzy)

라즈베리파이5 2대 + TurtleBot3 Waffle + Intel RealSense D435i 로 **라이다 SLAM, 카메라 vSLAM,
YOLO 객체인식**을 구현한 과정 정리.

---

## 1. 하드웨어 구성 (2-Pi 분산)

| | 역할 | 주요 장치 |
|---|---|---|
| **Pi #1** (192.168.0.62, 온보드) | 로봇 구동 + 라이다 SLAM | TurtleBot3 Waffle(OpenCR, 모터), LDS-02 라이다 |
| **Pi #2** (192.168.0.63, 카메라 전용) | D435i vSLAM + (YOLO 보조) | Intel RealSense D435i (USB3) |
| **VMware** (192.168.0.59, x86) | 시각화 + YOLO | RViz2, rtabmap_viz, YOLO(ultralytics) |

- 세 머신 동일 `ROS_DOMAIN_ID=30`, **chrony 시계 동기화**(Pi#1=서버, Pi#2=클라이언트, ~2ms)
- CPU 분산 이유: Pi5 한 대로는 카메라+모터 시리얼+SLAM 동시 처리 시 과부하

---

## 2. D435i 카메라 셋업 (Pi #2)

### 2-1. librealsense2 SDK (소스 빌드)
arm64 에 apt 패키지가 없어 소스 빌드:
```bash
git clone https://github.com/realsenseai/librealsense.git
cd librealsense && git checkout v2.58.4
sudo ./scripts/setup_udev_rules.sh
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON \
         -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false
make -j4 && sudo make install && sudo ldconfig
```

### 2-2. realsense-ros (ROS 래퍼)
`~/D435i` colcon 워크스페이스에 realsense-ros(4.58.4) 빌드.

### 2-3. ★ 핵심 트러블 — "USB 리셋 폭주"의 진짜 원인
증상: `No such device` 수백만 회 + `Permission denied` → 카메라 스트리밍 불가.
원인은 **하드웨어가 아니라 udev 규칙 누락**이었음:
- Pi#2 의 `setup_udev_rules.sh` 가 `99-realsense-libusb.rules` 를 안 깔고 `99-tof.rules` 만 남김.
- 해결: Pi#1 의 `/etc/udev/rules.d/99-realsense-libusb.rules` 를 Pi#2 에 복사 →
  `sudo udevadm control --reload-rules && sudo udevadm trigger`.
- 추가로 `sudo usermod -aG video,plugdev chan` (/dev/video* 접근).

### 2-4. 카메라 실행 파라미터 (안정/성능 튜닝)
- `enable_infra1/2:=false` — IR 원본 스트림 끔 (USB 대역폭 절약, 연결끊김 방지). **깊이는 그대로 나옴**(IR 쌍으로 카메라 내부 계산).
- `align_depth.enable:=true` — depth 를 color 에 정합 (RGB-D / YOLO 3D투영용).
- `enable_gyro/accel:=true`, `unite_imu_method:=2` — IMU 통합 → `/camera/camera/imu`.
- 해상도 `424x240x6` — Pi 부하↓.
- `taskset -c 2,3` 로 카메라를 코어 2,3 에 격리(모터 시리얼 보호; 단독 Pi#2 에선 선택).

---

## 3. 카메라 vSLAM (RTAB-Map VIO)

패키지 `tb3_lidar_vslam`, 런치 `lidar_vslam.launch.py`.

구성: `imu_filter_madgwick`(raw IMU→자세) → `rgbd_odometry`(시각-관성 오도메트리) →
`rtabmap`(그래프 SLAM + 루프클로저). 지도 = `/rtabmap/map`(2D), **`/rtabmap/cloud_map`(3D 컬러 점군)**.

단독 vSLAM 실행 (Pi#2):
```bash
ros2 launch tb3_lidar_vslam lidar_vslam.launch.py \
  lidar:=false camera:=true vslam:=true \
  frame_id:=camera_link vslam_publish_tf:=true force_3dof:=false use_sim_time:=false
```
핵심 옵션 (삽질로 얻은 것):
- `frame_id:=camera_link` — 카메라 단독이면 로컬 프레임(크로스머신 TF 지연 회피).
- `vslam_publish_tf:=true` — 단독이면 RTAB-Map 이 자기 map→odom 소유 (라이다와 동시 실행 시엔 false, 프레임 충돌 방지).
- `force_3dof:=false` — 카메라 기울어진 평면에 2D 제약을 걸면 지도가 대칭/뒤틀림 → 끔.
- 보기: **rtabmap_viz** (프레임 충돌 무관) 또는 RViz `/rtabmap/cloud_map`.
- VIO quality 는 특징 많고 밝은 장면에서 높음(정상 200~470), 민무늬/저조도면 0 → 지도 안 그려짐.

지도 저장:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/vslam_map --ros-args -r map:=/rtabmap/map  # 2D
rtabmap-export --cloud ~/.ros/rtabmap.db   # 3D 점군 -> rtabmap_cloud.ply
```

---

## 4. 라이다 2D SLAM (Pi #1, 참고)

패키지 `tb3_lidar_vslam`, `slam_toolbox` 사용. LDS-02 는 스캔당 점 개수가 변동(234~239)하는데
slam_toolbox 는 고정 개수를 기대 → 스캔 거부. **`scan_fixer.py`** 가 angle 파라미터를 실제 점 개수에
맞춰 `/scan_fixed` 로 보정. slam_toolbox 는 **라이프사이클 노드**라 `configure`→`activate` 필요.
지도: `~/maps/lab_map`(.pgm/.posegraph).

> YOLO/vSLAM 단계에선 **라이다 불필요** (라이다 지도는 이미 완성). 라이다는 추후 Nav2 자율주행에서 재사용.

---

## 5. YOLO 객체인식 (VMware)

패키지 `tb3_yolo_perception`, 노드 `yolo_detect.py`.

- **YOLOv8n = COCO 80종 사전학습** → 사람/의자/노트북/문 등 **학습 없이 즉시** 인식 (배경학습·이상탐지 아님).
- 파이프라인: RGB → YOLO 2D 검출 → bbox 중심 depth 로 **3D 좌표 투영**(카메라 내부행렬 K) →
  `/yolo/image`(박스영상) + `/yolo/markers`(3D 구+라벨).
- VMware 에서 실행(x86, 빠름). ROS rclpy + ultralytics 공존 위해 **`--system-site-packages` venv**:
```bash
python3 -m venv --system-site-packages ~/yolo_venv
source ~/yolo_venv/bin/activate && pip install ultralytics
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=30
python3 yolo_detect.py
```
- 보기: `rqt_image_view /yolo/image`, RViz MarkerArray `/yolo/markers`(Fixed Frame=map).

---

## 6. 산출물
- 라이다 2D 지도: `~/maps/lab_map.pgm/.yaml` (+ `lab_graph` 이어매핑용)
- vSLAM 3D 점군: `~/maps/vslam_cloud.ply/.pcd` (27만 포인트), 2D: `vslam_map`
- RTAB-Map DB: `~/.ros/rtabmap.db` (재추출·이어매핑)
- 지도 보기(VMware): `pcl_viewer vslam_cloud.pcd`(3D), `eog vslam_map.pgm`(2D)
