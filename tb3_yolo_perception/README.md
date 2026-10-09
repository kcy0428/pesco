# tb3_yolo_perception

D435i RGB-D 로 **YOLOv8 객체인식 + 3D 위치추정** (ROS 2 Jazzy). VMware(x86)에서 실행 권장.

YOLOv8n 은 **COCO 80종 사전학습 모델** — 사람/의자/노트북/병/문/TV 등을 **학습 없이 바로** 인식.
(배경학습·이상탐지가 아님. 특수 물체는 나중에 파인튜닝.)

## 파이프라인
```
RGB + aligned_depth + camera_info
  -> YOLOv8n 2D 검출(bbox+클래스)
  -> bbox 중심 depth 로 3D 좌표 투영 (카메라 광학프레임)
  -> /yolo/image (박스 영상) + /yolo/markers (3D 마커: 구 + 라벨"이름 신뢰도 거리")
```

## 설치 (VMware, 1회)
ROS2 의 rclpy 와 ultralytics 를 함께 쓰려면 **system-site-packages venv** 필요:
```bash
sudo apt install -y python3.12-venv
python3 -m venv --system-site-packages ~/yolo_venv
source ~/yolo_venv/bin/activate
pip install ultralytics
```

## 실행
```bash
source /opt/ros/jazzy/setup.bash
source ~/yolo_venv/bin/activate
export ROS_DOMAIN_ID=30
python3 yolo_detect.py            # 첫 실행 시 yolov8n.pt 자동 다운로드
```
전제: Pi#2 에서 카메라(vSLAM) 가 돌아 `/camera/...` 토픽이 네트워크로 들어와야 함.

## 보기
```bash
ros2 run rqt_image_view rqt_image_view /yolo/image    # 박스 그려진 영상
# RViz: Add -> MarkerArray -> /yolo/markers
#   Fixed Frame=map (vSLAM 돌면 지도 위에 객체 표시) 또는 camera_color_optical_frame
```

## 파라미터
- `model` (기본 yolov8n.pt) — yolov8s/m 등으로 교체 가능(정확도↑ 속도↓)
- `conf` (기본 0.4) — 검출 신뢰도 임계값
- `rgb_topic` / `depth_topic` / `info_topic`

## 메모
- YOLO 는 무거워 x86 VMware 가 Pi(ARM) 보다 훨씬 빠름
- 마커는 camera_color_optical_frame 기준 → vSLAM TF 체인(map->...->optical) 있으면 지도 위에 정합
