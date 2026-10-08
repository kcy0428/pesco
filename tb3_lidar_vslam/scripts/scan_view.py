#!/usr/bin/env python3
"""
LiDAR /scan 실시간 시각화 — RViz 없이 터미널에서 matplotlib 창을 띄운다.
로봇을 중심으로 라이다가 '보는 영역'을 위에서 내려다본 평면도(x-y)로 그린다.

실행 (디스플레이 있는 PC = VMware 에서):
  export ROS_DOMAIN_ID=30
  python3 scan_view.py                 # 기본 /scan
  python3 scan_view.py --topic /scan   # 토픽 지정

의존성: python3-matplotlib, rclpy (ROS2). 없으면: sudo apt install python3-matplotlib
"""

import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt


class ScanView(Node):
    def __init__(self, topic):
        super().__init__('scan_view')
        self.msg = None
        self.create_subscription(LaserScan, topic, self._cb, qos_profile_sensor_data)
        self.get_logger().info(f'구독: {topic} (Best Effort). 창이 뜨면 로봇은 중앙 ▲, 전방=위')

    def _cb(self, msg):
        self.msg = msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/scan')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = ScanView(args.topic)

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    fig.canvas.manager.set_window_title('LiDAR view (/scan)')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            m = node.msg
            if m is None:
                continue
            n = len(m.ranges)
            ang = m.angle_min + np.arange(n) * m.angle_increment
            rng = np.asarray(m.ranges, dtype=float)
            valid = np.isfinite(rng) & (rng > m.range_min) & (rng < (m.range_max or 12.0))
            # 라이다 프레임: x=전방, y=좌측. 화면은 전방을 위로 보이게 (x→위, y→좌).
            xf = rng[valid] * np.cos(ang[valid])   # 전방
            yl = rng[valid] * np.sin(ang[valid])   # 좌측
            ax.clear()
            ax.scatter(-yl, xf, s=6, c='red')       # 화면 가로=-좌측(=우측+), 세로=전방
            ax.plot(0, 0, 'b^', markersize=12)      # 로봇
            # 실제 점 분포에 맞춰 축 자동 확대 (고정 ±12m 로 두면 작은 방이 중앙에 뭉쳐 보임)
            rmaxd = float(rng[valid].max()) if valid.any() else 1.0
            lim = max(1.0, rmaxd * 1.15)
            ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_title(f'LiDAR /scan — points:{valid.sum()}  (▲=robot, up=forward)')
            ax.set_xlabel('right (m)'); ax.set_ylabel('forward (m)')
            plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
