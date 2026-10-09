# pesco — D435i / TurtleBot3 SLAM 프로젝트

Raspberry Pi 5 (Ubuntu 24.04, ROS 2 **Jazzy**) + **TurtleBot3 Waffle** + **Intel RealSense D435i**
기반의 SLAM / 자율주행 작업공간입니다.

최종 목표: 지도를 만들고(SLAM), 음성(STT·LLM·TTS)으로 목적지를 말하면 Nav2로 자율주행하는
**음성 안내로봇**.

**현재까지 완성(2026-10-09):**
- ✅ **LiDAR 2D SLAM** (slam_toolbox) — 점유격자 지도 생성·저장
- ✅ **D435i Visual-Inertial SLAM** (RTAB-Map VIO) — 3D 컬러 점군 지도 생성·저장
- ✅ **YOLO 객체인식 + 3D 위치추정** — RGB-D 로 COCO 80종 검출, `/yolo/image`·`/yolo/markers`
- ✅ **Nav2 자율주행** (라이다 지도 기반, AMCL 위치추정) — 목표 좌표로 자율 이동
- ✅ **음성 안내로봇 end-to-end** — Gemini Live API(STT+LLM+TTS+함수호출)로 목적지를 말하면
  웨이포인트 조회→Nav2 자율주행. 등록 장소: 로비/회의실/화장실/정수기. 이동 중 YOLO 동시 실행.
- 구성: **2-Pi 분산**(Pi#1=로봇/라이다/Nav2, Pi#2=카메라/vSLAM) + VMware(시각화/YOLO/음성), chrony 시계동기화

> 남은 작업: 스피커(TTS 출력)는 로봇에 USB 스피커 장착 예정. YOLO 를 Pi#2 로 이전하면 VMware
> 부하 분산(아래 "알려진 이슈" 참고).

> 전체 구현 과정·트러블슈팅은 **[IMPLEMENTATION.md](IMPLEMENTATION.md)** 참고.

## 하드웨어
- Raspberry Pi 5 (Ubuntu 24.04 arm64)
- TurtleBot3 Waffle (OpenCR + Dynamixel XM430, 360° LDS 라이다)
- Intel RealSense D435i (USB3 연결)

## 필요한 패키지

### 1) 시스템 의존성 (apt 아님 — 소스 빌드)
- **librealsense2 (≥ 2.58)** — RealSense SDK. arm64에는 안정적 apt 패키지가 없어 소스 빌드:
  ```bash
  git clone https://github.com/realsenseai/librealsense.git
  cd librealsense && git checkout v2.58.4
  sudo ./scripts/setup_udev_rules.sh
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON \
           -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false
  make -j2 && sudo make install && sudo ldconfig
  ```
  (OpenGL 링크 에러 시 `sudo apt install libglvnd-dev libopengl-dev`)

### 2) ROS 2 패키지 (apt)
```bash
sudo apt install -y \
  ros-jazzy-depthimage-to-laserscan \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-nav2-bringup \
  ros-jazzy-rviz2
```

### 3) 소스 워크스페이스
- **realsense-ros** (이 저장소의 서브모듈, v4.58.4) — D435i ROS 래퍼
- **turtlebot3** (`~/turtlebot3_ws`) — 로봇 bringup / 라이다 / URDF
- **ros2_ws** (`~/ros2_ws`) — 사용자 패키지(assisted_teleop, lidar_driving, shared_control)

## 빌드
```bash
source /opt/ros/jazzy/setup.bash
cd ~/D435i
rosdep install --from-paths realsense-ros --ignore-src -r -y
colcon build --cmake-args -DUSE_LIFECYCLE_NODE=OFF
source install/setup.bash
```

## 실행 (병렬 SLAM 비교)

| 역할 | 센서 | TF(map→odom) | 시각화 |
|------|------|--------------|--------|
| LiDAR SLAM (slam_toolbox) | LDS 라이다 `/scan` | **소유** | RViz2 |
| Visual SLAM (RTAB-Map) | D435i RGB-D | 발행 안 함(`publish_tf:=false`) | rtabmap_viz |

```bash
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=30

# 1) 로봇 + 라이다 + odom
ros2 launch turtlebot3_bringup robot.launch.py
# 2) D435i 카메라 (pointcloud 포함)  — launch/d435i_slam.launch.py 참고
# 3) slam_toolbox (라이다) / 4) RTAB-Map (카메라)  — dual_slam.launch.py (작성 예정)
# 5) 운전
ros2 run turtlebot3_teleop teleop_keyboard
```

## YOLO 객체인식 (시맨틱 맵)

`tb3_yolo_perception` — D435i RGB-D 로 YOLOv8(COCO 80종 사전학습) 검출 + depth 3D 투영.
`/yolo/image`(박스영상) + `/yolo/markers`(3D 객체 마커) 발행.
```bash
# (1회 설치) rclpy + ultralytics 공존 위해 --system-site-packages venv
python3 -m venv --system-site-packages ~/yolo_venv
source ~/yolo_venv/bin/activate
pip install ultralytics "numpy<2" "opencv-python<5"   # ROS(numpy1.x) 호환 위해 버전 고정
# 실행
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=30
python3 ~/D435i/tb3_yolo_perception/scripts/yolo_detect.py
# 보기: rqt_image_view /yolo/image, RViz MarkerArray /yolo/markers (Fixed Frame=map)
```

## Nav2 자율주행 + 음성 안내 (`tb3_voice_guide`)

라이다 지도 기반 Nav2 자율주행 + Gemini Live 음성 명령. **목적지를 말하면** 등록 웨이포인트로
자율 이동한다. (TTS 음성응답은 스피커 장착 후 활성화 — 현재 STT·주행은 동작)
```bash
# Pi#1: 로봇 + 라이다
ros2 launch turtlebot3_bringup robot.launch.py
# Pi#1: Nav2 (저장한 라이다 지도로 localization+navigation)
ros2 launch nav2_bringup bringup_launch.py \
  map:=$HOME/maps/lab_map.yaml use_sim_time:=false \
  params_file:=/opt/ros/jazzy/share/turtlebot3_navigation2/param/waffle.yaml
# (RViz 에서 2D Pose Estimate 로 초기 위치 지정 → AMCL 수렴)

# 웨이포인트 등록 — 로봇을 원하는 곳에 두고 현재 위치 저장
python3 ~/D435i/tb3_voice_guide/scripts/save_waypoint.py 로비 \
  --file ~/D435i/tb3_voice_guide/config/waypoints.yaml

# VMware: 음성 노드 (마이크 + 인터넷 필요)
source ~/voice_venv/bin/activate          # google-genai, sounddevice, pyyaml, numpy<2
export GEMINI_API_KEY="..."               # ~/.bashrc 에 등록 (git 에 커밋 금지)
python3 ~/D435i/tb3_voice_guide/scripts/voice_guide.py \
  --waypoints ~/D435i/tb3_voice_guide/config/waypoints.yaml
# 말하기 예: "회의실 가줘" → navigate_to(회의실) → Nav2 자율주행
```

## 이 저장소 구성
- `IMPLEMENTATION.md` — **전체 구현 정리** (카메라 셋업 → vSLAM → YOLO, 트러블슈팅)
- `CLAUDE.md` — 작업공간 가이드(빌드/구조/파라미터 규약)
- **`tb3_lidar_vslam/`** — LiDAR SLAM + D435i vSLAM 통합 패키지 (scan_fixer, scan_view, 런치/설정)
- **`tb3_pcl_perception/`** — D435i 포인트클라우드 3D 인지 파이프라인
- **`tb3_yolo_perception/`** — YOLO 객체인식 + 3D 투영 노드
- **`tb3_voice_guide/`** — Gemini Live 음성 안내 + Nav2 자율주행 (voice_guide, save_waypoint, waypoints.yaml)
- `launch/` — 초기 통합 런치들 (d435i_slam, dual_slam, vslam_vio)
- `config/` — RViz 설정, 수정한 waffle URDF 참고본
- `realsense-ros/` — RealSense ROS 래퍼 (서브모듈)
- (빌드 산출물 `build/ install/ log/`, 참고용 `turtlebot3-autonomy-stack/` 는 `.gitignore`로 제외)

## 중요 메모 / 알려진 이슈
- **arm64 pointcloud 파라미터 이름은 `pointcloud__neon_.enable`** (NEON 접미사). `pointcloud.enable` 아님.
- **RealSense 토픽 QoS = Best Effort.** RViz에서 이미지/포인트클라우드는 Reliability를 Best Effort로.
- **D435i 장착 위치**: TurtleBot3 Waffle URDF(`turtlebot3_waffle.urdf`)의 `camera_joint` origin을
  `xyz="0.09 0 0.12"` (상판 중앙·라이다 바로 앞, 정면)으로 수정함. URDF 주석에 콜론(`:`)을 넣으면
  `robot_description` YAML 파싱이 깨지므로 금지.
- **모터 통신 이슈**: `Failed transmit instruction packet` 발생 시 LiPo 배터리 충전 / OpenCR 리셋 /
  모터 케이블 점검. (단일 Pi 에 카메라까지 물리면 CPU 과부하로 시리얼이 굶어 발생 → 카메라를 Pi#2 로 분리)
- **VMware CPU 병목(음성 지연)**: VMware 가 2코어뿐이라 YOLO+RViz+음성을 동시에 돌리면 load 가
  3.5 넘게 치솟아(코어당 1.75) 음성 노드의 asyncio 마이크 전송 루프가 밀려 **STT 응답이 느려짐**
  (인식 자체는 정상 — 연동 끊김 아님. 참고로 "VMware 연동 안 됨"은 STT 가 아니라 TTS/스피커 문제).
  (Pi#2 는 카메라만이라 load 0.3 수준으로 한가함.) 해결: ① VMware 코어 4개로 ② RViz 끄기
  ③ **YOLO 를 한가한 Pi#2 로 이전**(네트워크로 카메라 영상 안 넘겨 WiFi 부담도↓, 권장).
- **포인트클라우드 3D 뷰**: 라이브 점군은 `pointcloud__neon_.enable:=true` 로 켜고 RViz 에서
  Color Transformer=`AxisColor`, Axis=`Z` + 시점을 비스듬히 기울이면 높이(z)가 색으로 보임.
- **Gemini Live 모델명**: `gemini-2.5-flash-native-audio-latest` (AUDIO 응답 + function calling 확인).
  API 키는 VMware `~/.bashrc` 의 `GEMINI_API_KEY`; **`.env`/`*.key` 와 함께 git 제외**.
