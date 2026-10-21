#!/usr/bin/env python3
"""Drive the tugger a specified distance using wheel odometry."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class DistanceDriver(Node):
    def __init__(self, distance, speed):
        super().__init__('tugger_distance_driver')

        self.target = abs(distance)
        self.direction = 1.0 if distance >= 0 else -1.0
        self.speed = speed

        self.wheel_radius = 0.055
        self.tolerance = 0.002

        self.start_pose = None
        self.current_pose = None
        self.last_progress = None
        self.last_progress_time = time.monotonic()

        self.traction_pub = self.create_publisher(
            Float64MultiArray,
            '/traction_controller/commands',
            10
        )

        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            10
        )

        self.create_subscription(
            Odometry,
            '/wheel/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.current_pose = (p.x, p.y)

        if self.start_pose is None:
            self.start_pose = self.current_pose
            self.last_progress = 0.0

    def command_wheels(self, angular_speed):
        msg = Float64MultiArray()
        msg.data = [angular_speed, angular_speed]
        self.traction_pub.publish(msg)

    def straighten(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0]
        self.steering_pub.publish(msg)

    def stop(self):
        for _ in range(5):
            self.command_wheels(0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        self.get_logger().info(
            f'Target: {self.direction * self.target:.3f} m'
        )

        while rclpy.ok() and self.current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.straighten()

        start_time = time.monotonic()
        last_odom_time = time.monotonic()

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

                if self.current_pose is None:
                    continue

                dx = self.current_pose[0] - self.start_pose[0]
                dy = self.current_pose[1] - self.start_pose[1]

                traveled = math.hypot(dx, dy)
                remaining = self.target - traveled

                if remaining <= self.tolerance:
                    self.get_logger().info(
                        f'Target reached: {traveled:.3f} m'
                    )
                    return True

                # Slow down near the target.
                linear_speed = min(
                    self.speed,
                    max(0.005, remaining * 0.5)
                )

                wheel_speed = (
                    self.direction * linear_speed / self.wheel_radius
                )

                self.command_wheels(wheel_speed)

                now = time.monotonic()

                if (
                    self.last_progress is None
                    or traveled > self.last_progress + 0.005
                ):
                    self.last_progress = traveled
                    self.last_progress_time = now

                if now - self.last_progress_time > 8.0:
                    self.get_logger().error(
                        'Movement stalled. Stopping.'
                    )
                    return False

                if now - start_time > 120.0:
                    self.get_logger().error(
                        'Safety timeout. Stopping.'
                    )
                    return False

        finally:
            self.stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--distance', type=float, required=True,
        help='Signed travel distance in meters'
    )
    parser.add_argument(
        '--speed', type=float, default=0.08,
        help='Maximum linear speed in m/s'
    )

    args = parser.parse_args()

    if not math.isfinite(args.distance):
        parser.error('Distance must be finite')

    if not math.isfinite(args.speed) or args.speed <= 0:
        parser.error('Speed must be positive and finite')

    rclpy.init()

    node = DistanceDriver(args.distance, args.speed)

    try:
        success = node.run()
    except KeyboardInterrupt:
        node.stop()
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
