#!/usr/bin/env python3
"""
YOLOv8 객체인식 + 3D 위치추정 (D435i).

파이프라인:
  RGB(/camera/.../color) + aligned_depth(/camera/.../aligned_depth_to_color) + camera_info
    -> YOLOv8n(사전학습 COCO 80종) 2D 검출
    -> bbox 중심의 depth 로 3D 좌표 투영 (카메라 광학프레임)
    -> 결과: 주석 이미지(/yolo/image) + 3D 마커(/yolo/markers, RViz 에서 지도 위에 표시)

실행 (VMware, venv + ROS):
  source /opt/ros/jazzy/setup.bash
  source ~/yolo_venv/bin/activate   # 또는 ~/yolo_venv/bin/python 로 직접
  export ROS_DOMAIN_ID=30
  python3 yolo_detect.py

보기:
  ros2 run rqt_image_view rqt_image_view /yolo/image      # 박스 그려진 영상
  RViz: Add -> MarkerArray -> /yolo/markers (Fixed Frame=map, vSLAM 돌고 있을 때 지도 위에 뜸)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
import message_filters
from ultralytics import YOLO

# cv_bridge 미사용 (ROS numpy<2 vs ultralytics opencv/numpy>=2 충돌 회피) — numpy 로 직접 변환


class YoloDetect(Node):
    def __init__(self):
        super().__init__('yolo_detect')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('conf', 0.4)
        self.declare_parameter('rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('info_topic', '/camera/camera/color/camera_info')

        self.conf = float(self.get_parameter('conf').value)
        self.model = YOLO(self.get_parameter('model').value)  # 첫 실행 시 자동 다운로드
        self.names = self.model.names
        self.K = None

        self.create_subscription(CameraInfo, self.get_parameter('info_topic').value,
                                 self._info_cb, 10)
        rgb = message_filters.Subscriber(self, Image, self.get_parameter('rgb_topic').value,
                                         qos_profile=qos_profile_sensor_data)
        depth = message_filters.Subscriber(self, Image, self.get_parameter('depth_topic').value,
                                           qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([rgb, depth], 10, 0.1)
        self.sync.registerCallback(self._cb)

        self.img_pub = self.create_publisher(Image, '/yolo/image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/yolo/markers', 10)
        self.get_logger().info('yolo_detect 시작: RGB->YOLO->depth 3D투영. /yolo/image, /yolo/markers 발행')

    def _info_cb(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def _depth_at(self, depth, cx, cy, win=4):
        """bbox 중심 주변 창의 유효 depth 중앙값(m). depth 는 16UC1(mm) 가정."""
        h, w = depth.shape[:2]
        y0, y1 = max(0, cy - win), min(h, cy + win)
        x0, x1 = max(0, cx - win), min(w, cx + win)
        patch = depth[y0:y1, x0:x1].astype(np.float32)
        vals = patch[(patch > 0) & np.isfinite(patch)]
        if vals.size == 0:
            return 0.0
        return float(np.median(vals)) / 1000.0  # mm -> m

    def _cb(self, rgb_msg, depth_msg):
        if self.K is None:
            return
        # --- numpy 직접 디코드 (cv_bridge 미사용) ---
        arr = np.frombuffer(rgb_msg.data, np.uint8).reshape(rgb_msg.height, rgb_msg.width, 3)
        bgr = arr[:, :, ::-1] if rgb_msg.encoding == 'rgb8' else arr   # ultralytics=BGR
        bgr = np.ascontiguousarray(bgr)
        depth = np.frombuffer(depth_msg.data, np.uint16).reshape(depth_msg.height, depth_msg.width)

        res = self.model(bgr, conf=self.conf, verbose=False)[0]
        annotated = res.plot()

        fx, fy, cx0, cy0 = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        markers = MarkerArray()
        for i, box in enumerate(res.boxes):
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            d = self._depth_at(depth, cx, cy)
            if d <= 0:
                continue
            X = (cx - cx0) * d / fx
            Y = (cy - cy0) * d / fy
            Z = d
            name = self.names[int(box.cls[0])]
            conf = float(box.conf[0])

            sphere = Marker()
            sphere.header = rgb_msg.header   # camera_color_optical_frame
            sphere.ns, sphere.id = 'yolo', i
            sphere.type, sphere.action = Marker.SPHERE, Marker.ADD
            sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = X, Y, Z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
            sphere.color.a, sphere.color.r, sphere.color.g = 1.0, 1.0, 0.2
            sphere.lifetime = Duration(seconds=0.5).to_msg()
            markers.markers.append(sphere)

            label = Marker()
            label.header = rgb_msg.header
            label.ns, label.id = 'yolo_label', 1000 + i
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = X, Y, Z + 0.15
            label.pose.orientation.w = 1.0
            label.scale.z = 0.12
            label.color.a = label.color.r = label.color.g = label.color.b = 1.0
            label.text = f'{name} {conf:.2f} {d:.1f}m'
            label.lifetime = Duration(seconds=0.5).to_msg()
            markers.markers.append(label)

        self.marker_pub.publish(markers)
        # --- 주석 영상 직접 인코드 (annotated = BGR) ---
        annotated = np.ascontiguousarray(annotated)
        out = Image()
        out.header = rgb_msg.header
        out.height, out.width = annotated.shape[:2]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = annotated.tobytes()
        self.img_pub.publish(out)


def main():
    rclpy.init()
    node = YoloDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
