# 구현 정리 — 음성 안내로봇 (D435i vSLAM + YOLO + Nav2, TurtleBot3 / ROS 2 Jazzy)

라즈베리파이5 2대 + TurtleBot3 Waffle + Intel RealSense D435i 로 **라이다 SLAM, 카메라 vSLAM,
YOLO 객체인식, Nav2 자율주행, 음성 안내(Gemini Live)**를 구현한 과정 정리.
**이 문서만 보고 처음부터 재현**할 수 있도록 순서대로 정리했다.

### 재현 순서 한눈에
1. [사전 준비](#0-사전-준비-ospackage) — OS/ROS/저장소
2. [D435i 카메라 셋업](#2-d435i-카메라-셋업-pi-2) (Pi#2)
3. [카메라 vSLAM](#3-카메라-vslam-rtab-map-vio) → 3D 점군 지도
4. [라이다 2D SLAM](#4-라이다-2d-slam-pi-1-참고) (Pi#1)
5. [YOLO 객체인식](#5-yolo-객체인식-vmware) (VMware)
6. [Nav2 자율주행 + 음성 안내](#6-nav2-자율주행--음성-안내-gemini-live) — 목적지 말하면 자율 이동
7. [전체 실행 순서](#8-전체-실행-순서-복붙용)

---

## 0. 사전 준비 (OS/package)

**세 머신 공통:** Ubuntu 24.04 + ROS 2 Jazzy, `~/.bashrc` 에:
```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc   # 로봇 Pi 만
```
- Pi#1(로봇): `ros-jazzy-turtlebot3*`, `ros-jazzy-slam-toolbox`, `ros-jazzy-nav2-bringup`
- Pi#2(카메라): librealsense2(§2-1) + realsense-ros, `ros-jazzy-rtabmap-ros`,
  `ros-jazzy-imu-filter-madgwick`
- VMware(PC): `ros-jazzy-rviz2`, `ros-jazzy-rqt-image-view`, `pcl-tools`, YOLO venv(§5)

**저장소(프로젝트 패키지):**
```bash
git clone --recursive git@github.com:kcy0428/pesco.git ~/D435i   # realsense-ros 서브모듈 포함
cd ~/D435i && colcon build && source install/setup.bash
```

**두 Pi 시계 동기화(chrony)** — 분산 SLAM 타임스탬프 정합:
```bash
sudo apt install -y chrony
# Pi#1 (서버): /etc/chrony/chrony.conf 에  allow 192.168.0.0/24  +  local stratum 10
# Pi#2 (클라): /etc/chrony/chrony.conf 에  server <PI1_IP> iburst prefer
sudo systemctl restart chrony   # 양쪽.  확인: chronyc sources (^* 표시 = 동기화됨)
```

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
- **numpy ABI 함정**: ROS(rclpy)는 numpy 1.x, 최신 ultralytics/opencv 는 numpy 2.x 를 끌어와 충돌.
  → `pip install ultralytics "numpy<2" "opencv-python<5"` 로 고정. 노드는 `cv_bridge` 없이
  numpy 로 직접 디코드(`np.frombuffer(...).reshape(h,w,3)`)해 의존성 충돌을 아예 피함.

---

## 6. Nav2 자율주행 + 음성 안내 (Gemini Live)

패키지 `tb3_voice_guide`. 흐름: **🎤 마이크 → Gemini Live(STT+LLM+TTS+함수호출) →
`navigate_to(목적지)` → 웨이포인트 조회(이름→map 좌표) → Nav2 `NavigateToPose` 목표 전송**.

### 6-1. Nav2 (라이다 지도 기반 자율주행, Pi#1)
`rviz2` 없는 헤드리스 Pi 에서는 `navigation2.launch.py` 가 `rviz2` 패키지를 찾다 실패 →
**`nav2_bringup bringup_launch.py` + turtlebot3 `waffle.yaml` 파라미터**로 실행:
```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=$HOME/maps/lab_map.yaml use_sim_time:=false \
  params_file:=/opt/ros/jazzy/share/turtlebot3_navigation2/param/waffle.yaml
```
- **초기 위치**: RViz(VMware)에서 `2D Pose Estimate` 로 로봇 실제 위치를 찍어야 AMCL 이 수렴.
  (실수로 `Nav2 Goal`/`2D Pose Estimate` 를 잘못 찍으면 지도상 로봇이 튐 → 다시 찍어 정렬.)
- 검증: `/navigate_to_pose` 액션 서버 존재, 목표 전송 시 로봇 실제 주행 + 최종 status 4(SUCCEEDED).
  (CLI `send_goal` 이 타임아웃돼도 로봇은 실제로 도착하는 경우 있음 — 주행 자체가 기준.)

### 6-2. 웨이포인트 등록 (`save_waypoint.py`)
로봇을 원하는 장소에 두고 **현재 map→base_footprint TF 를 읽어 이름으로 저장**:
```bash
python3 ~/D435i/tb3_voice_guide/scripts/save_waypoint.py 로비 \
  --aliases "입구,현관" --file ~/D435i/tb3_voice_guide/config/waypoints.yaml
```
등록된 장소(`config/waypoints.yaml`): **로비 / 회의실 / 화장실 / 정수기** (각 별칭 포함).

### 6-3. 음성 노드 (`voice_guide.py`, VMware)
- 모델 **`gemini-2.5-flash-native-audio-latest`** (Live 네이티브 음성 — AUDIO 응답 + function
  calling 동시 지원. `gemini-2.0-flash-live-001` 은 미존재 → `models.list` 로 확인해 교체함).
- `response_modalities=["AUDIO"]` + `input/output_audio_transcription`(디버그 전사 로그).
- **마이크 자동선택**: 기본 입력이 가상 사운드카드면 무음 → 노드가 `c920/webcam/usb` 장치를 찾아 선택.
- venv: `google-genai, sounddevice, pyyaml, numpy<2` (+ 시스템 `libportaudio2`). 키는 `GEMINI_API_KEY`.
```bash
source ~/voice_venv/bin/activate
export GEMINI_API_KEY="..."     # ~/.bashrc 등록, git 커밋 금지(.env/*.key 는 .gitignore)
python3 ~/D435i/tb3_voice_guide/scripts/voice_guide.py \
  --waypoints ~/D435i/tb3_voice_guide/config/waypoints.yaml
# "회의실 가줘" → [🎤 들림]/[🔊 응답]/[🚗 navigate_to] 로그 → Nav2 주행
```
- **TTS 출력(스피커)**: 게스트 오디오는 Ensoniq 가상카드로 재생되나 VMware→Windows 호스트 스피커
  라우팅이 막혀 소리가 안 남. **로봇에 USB 스피커 장착 후 활성화 예정**(STT·주행은 정상 동작).

### 6-4. ★ 성능 교훈 — VMware 2코어 병목
YOLO + RViz + 음성을 **VMware(2코어)에서 동시 실행**하면 load≈3.5(코어당 1.75)로 과부하 →
음성 노드의 **asyncio 마이크 전송 루프가 밀려 STT 응답이 느려짐**(인식·연동은 정상, 지연일 뿐).
(Pi#2 는 카메라만이라
load 0.3 으로 한가.) 대책: ① VMware 코어 4개로 ② RViz 끄기 ③ **YOLO 를 Pi#2 로 이전**
(카메라 영상이 네트워크를 안 건너가 WiFi 부담도↓ — 권장).

---

## 7. 산출물
- 라이다 2D 지도: `~/maps/lab_map.pgm/.yaml` (+ `lab_graph` 이어매핑용)
- vSLAM 3D 점군: `~/maps/vslam_cloud.ply/.pcd` (27만 포인트), 2D: `vslam_map`
- RTAB-Map DB: `~/.ros/rtabmap.db` (재추출·이어매핑)
- 지도 보기(VMware): `pcl_viewer vslam_cloud.pcd`(3D), `eog vslam_map.pgm`(2D)

---

## 8. 전체 실행 순서 (복붙용)

모든 터미널 `export ROS_DOMAIN_ID=30`. (한 번에 다 돌리지 말고 목적에 맞게 선택)

### A. 라이다 2D 지도 만들기 (Pi#1)
```bash
# 터미널1: 로봇
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_bringup robot.launch.py
# 터미널2: 라이다 SLAM (scan_fixer 포함) + 라이프사이클 활성화
ros2 launch tb3_lidar_vslam lidar_vslam.launch.py lidar:=true camera:=false vslam:=false
ros2 lifecycle set /slam_toolbox configure && ros2 lifecycle set /slam_toolbox activate
# 터미널3: 운전
ros2 run turtlebot3_teleop teleop_keyboard
# 완성 후 저장
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map
```

### B. D435i vSLAM 3D 지도 (Pi#2 단독)
```bash
ros2 launch tb3_lidar_vslam lidar_vslam.launch.py \
  lidar:=false camera:=true vslam:=true \
  frame_id:=camera_link vslam_publish_tf:=true force_3dof:=false
# VMware 보기: ros2 run rtabmap_viz rtabmap_viz   (또는 RViz /rtabmap/cloud_map)
# 저장(Pi#2): ros2 run nav2_map_server map_saver_cli -f ~/maps/vslam_map --ros-args -r map:=/rtabmap/map
#            rtabmap-export --cloud ~/.ros/rtabmap.db   # 3D ply
```

### C. YOLO 객체인식 (VMware, B가 돌아 카메라 토픽이 있을 때)
```bash
source ~/yolo_venv/bin/activate
python3 ~/D435i/tb3_yolo_perception/scripts/yolo_detect.py
# 보기: rqt_image_view /yolo/image  |  RViz MarkerArray /yolo/markers (Fixed Frame=map)
```

> 검증 포인트: SLAM 지도는 `/map` 또는 `/rtabmap/map` width/height 가 커지면 OK.
> vSLAM VIO 는 로그의 `Odom: quality=` 가 >0(보통 200~470)이어야 추적 중.
> YOLO 는 `/yolo/image`, `/yolo/markers` 가 ~5Hz 발행되면 OK.
