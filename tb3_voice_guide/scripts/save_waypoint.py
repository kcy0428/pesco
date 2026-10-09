#!/usr/bin/env python3
"""
웨이포인트 저장 — 로봇을 원하는 장소에 둔 뒤 현재 map 좌표를 이름으로 저장.

사용:
  python3 save_waypoint.py 로비 --file ~/D435i/tb3_voice_guide/config/waypoints.yaml
  python3 save_waypoint.py 회의실 --aliases "미팅룸,회의하는 곳" --file ...

동작: TF map->base_footprint 로 현재 로봇 위치(x,y,yaw)를 읽어 YAML 의 waypoints: 아래에 추가/갱신.
전제: Nav2(AMCL)가 돌아 map->odom 위치추정이 되고 있어야 함.
"""
import sys
import time
import math
import argparse

import yaml
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Saver(Node):
    def __init__(self):
        super().__init__('waypoint_saver')
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

    def get_pose(self, timeout=6.0):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                tr = t.transform.translation
                return tr.x, tr.y, quat_to_yaw(t.transform.rotation)
            except Exception:
                continue
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('name', help='장소 이름 (예: 로비)')
    ap.add_argument('--file', default='waypoints.yaml')
    ap.add_argument('--aliases', default='', help='쉼표구분 별칭 (예: "미팅룸,회의실")')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Saver()
    pose = node.get_pose()
    node.destroy_node()
    rclpy.shutdown()

    if pose is None:
        print('ERROR: map->base_footprint TF 없음. Nav2(AMCL) 위치추정이 되는지 확인하세요.')
        sys.exit(1)

    x, y, yaw = pose
    try:
        with open(args.file) as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        data = {}
    data.setdefault('waypoints', {})

    entry = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
    if args.aliases:
        entry['aliases'] = [s.strip() for s in args.aliases.split(',') if s.strip()]
    data['waypoints'][args.name] = entry

    with open(args.file, 'w') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print(f"저장됨: '{args.name}'  x={x:.3f} y={y:.3f} yaw={yaw:.3f}({math.degrees(yaw):.0f}deg)"
          f"  -> {args.file}")


if __name__ == '__main__':
    main()
