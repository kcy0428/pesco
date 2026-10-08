#!/usr/bin/env python3
"""
LaserScan 점 개수 보정 중계 노드.

LDS-02(ld08) 라이다는 스캔마다 점 개수가 변하는데(234~239), slam_toolbox 는
angle_min/max/increment 로 계산한 '기대 개수(238)'와 다르면 스캔을 거부한다
("LaserRangeScan contains N readings, expected M"). 그 결과 지도가 안 그려진다.

이 노드는 /scan 을 받아 angle_max 를 '실제 점 개수'에 맞게 재계산해서 /scan_fixed 로 내보낸다.
→ slam_toolbox 가 /scan_fixed 를 구독하면 기대 개수 == 실제 개수 가 되어 모든 스캔을 수용.

실행: ros2 run tb3_lidar_vslam scan_fixer.py   (또는 lidar_vslam.launch.py 가 자동 실행)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFixer(Node):
    def __init__(self):
        super().__init__('scan_fixer')
        self.pub = self.create_publisher(LaserScan, '/scan_fixed', qos_profile_sensor_data)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)
        self.get_logger().info('scan_fixer: /scan -> /scan_fixed (점 개수에 맞춰 angle_max 보정)')

    def cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n >= 2 and msg.angle_increment != 0.0:
            # angle_max 를 실제 점 개수에 맞게 재설정 → expected == n
            msg.angle_max = msg.angle_min + msg.angle_increment * (n - 1)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
