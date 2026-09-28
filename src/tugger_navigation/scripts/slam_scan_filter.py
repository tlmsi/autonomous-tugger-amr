#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class SlamScanFilter(Node):

    def __init__(self):
        super().__init__('slam_scan_filter')

        # Exact tugger body footprint in base_footprint.
        self.x_min = -0.45
        self.x_max =  0.45
        self.y_min = -0.30
        self.y_max =  0.30

        # front_left_lidar_link relative to base_footprint.
        self.sensor_x = 0.410
        self.sensor_y = 0.260
        self.sensor_yaw = 0.0

        self.subscription = self.create_subscription(
            LaserScan,
            '/front_left/scan',
            self.scan_callback,
            10
        )

        self.publisher = self.create_publisher(
            LaserScan,
            '/front_left/scan_slam',
            10
        )

        self.get_logger().info(
            'SLAM scan footprint filter started: '
            '/front_left/scan -> /front_left/scan_slam'
        )

        self.get_logger().info(
            'Filtering laser endpoints inside '
            'x=[-0.45,+0.45], y=[-0.30,+0.30]'
        )

    def scan_callback(self, msg):

        filtered = LaserScan()

        filtered.header = msg.header

        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment

        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time

        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max

        ranges = list(msg.ranges)
        removed = 0

        for i, r in enumerate(ranges):

            if not math.isfinite(r):
                continue

            angle_sensor = (
                msg.angle_min +
                i * msg.angle_increment
            )

            angle_base = (
                self.sensor_yaw +
                angle_sensor
            )

            x = (
                self.sensor_x +
                r * math.cos(angle_base)
            )

            y = (
                self.sensor_y +
                r * math.sin(angle_base)
            )

            inside_robot = (
                self.x_min <= x <= self.x_max and
                self.y_min <= y <= self.y_max
            )

            if inside_robot:
                ranges[i] = float('inf')
                removed += 1

        filtered.ranges = ranges
        filtered.intensities = list(msg.intensities)

        self.publisher.publish(filtered)


def main(args=None):

    rclpy.init(args=args)

    node = SlamScanFilter()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
