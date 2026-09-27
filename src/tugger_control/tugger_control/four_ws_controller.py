#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class FourWSController(Node):

    def __init__(self):
        super().__init__('four_ws_controller')

        # Tugger geometry
        self.declare_parameter('wheelbase', 0.58)
        self.declare_parameter('track_width', 0.44)
        self.declare_parameter('wheel_radius', 0.055)
        self.declare_parameter('max_steering_angle', 0.7854)

        self.wheelbase = float(
            self.get_parameter('wheelbase').value
        )
        self.track_width = float(
            self.get_parameter('track_width').value
        )
        self.wheel_radius = float(
            self.get_parameter('wheel_radius').value
        )
        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value
        )

        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            10
        )

        self.traction_pub = self.create_publisher(
            Float64MultiArray,
            '/traction_controller/commands',
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info(
            '4WS controller started '
            f'(L={self.wheelbase:.3f} m, '
            f'W={self.track_width:.3f} m, '
            f'R={self.wheel_radius:.3f} m)'
        )

    def cmd_vel_callback(self, msg):

        v = float(msg.linear.x)
        yaw_rate = float(msg.angular.z)

        # ----------------------------------------------------------
        # STOP / NO LONGITUDINAL MOTION
        #
        # This vehicle is not differential drive and cannot perform
        # a zero-radius rotation.
        # ----------------------------------------------------------

        if abs(v) < 1e-4:
            self.publish_commands(
                front_angle=0.0,
                rear_angle=0.0,
                rear_left_speed=0.0,
                rear_right_speed=0.0
            )
            return

        # ----------------------------------------------------------
        # SYMMETRIC COUNTER-PHASE 4WS
        #
        # Front axle:
        #   FL = FR = +delta
        #
        # Rear axle:
        #   RL = RR = -delta
        #
        # For symmetric 4WS:
        #
        #   yaw_rate = (2*v/L) * tan(delta)
        #
        # therefore:
        #
        #   delta = atan(yaw_rate * L / (2*v))
        # ----------------------------------------------------------

        delta = math.atan(
            (yaw_rate * self.wheelbase) / (2.0 * v)
        )

        delta = max(
            -self.max_steering_angle,
            min(self.max_steering_angle, delta)
        )

        front_angle = delta
        rear_angle = -delta

        # ----------------------------------------------------------
        # REAR TRACTION DIFFERENTIAL
        #
        # During a turn the left and right wheels follow different
        # radii. Therefore their longitudinal speeds cannot remain
        # equal.
        #
        #   v_left  = v - yaw_rate * W/2
        #   v_right = v + yaw_rate * W/2
        #
        # Convert linear wheel speeds [m/s] to angular speeds [rad/s].
        # ----------------------------------------------------------

        rear_left_linear = (
            v - yaw_rate * self.track_width / 2.0
        )

        rear_right_linear = (
            v + yaw_rate * self.track_width / 2.0
        )

        rear_left_speed = (
            rear_left_linear / self.wheel_radius
        )

        rear_right_speed = (
            rear_right_linear / self.wheel_radius
        )

        self.publish_commands(
            front_angle,
            rear_angle,
            rear_left_speed,
            rear_right_speed
        )

    def publish_commands(
        self,
        front_angle,
        rear_angle,
        rear_left_speed,
        rear_right_speed
    ):

        steering_msg = Float64MultiArray()

        # Physical steering architecture:
        # one steering position per axle.
        steering_msg.data = [
            front_angle,   # FL
            front_angle,   # FR
            rear_angle,    # RL
            rear_angle     # RR
        ]

        traction_msg = Float64MultiArray()

        # Independent rear traction motors.
        traction_msg.data = [
            rear_left_speed,
            rear_right_speed
        ]

        self.steering_pub.publish(steering_msg)
        self.traction_pub.publish(traction_msg)


def main(args=None):
    rclpy.init(args=args)

    node = FourWSController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
