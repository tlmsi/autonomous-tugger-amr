#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray


class FourWSController(Node):

    def __init__(self):
        super().__init__('four_ws_controller')

        # Vehicle geometry
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
            TwistStamped,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Safety watchdog.
        # If cmd_vel disappears, stop the vehicle automatically.
        self.cmd_vel_timeout = 0.5
        self.last_cmd_vel_time = None
        self.watchdog_triggered = False

        self.watchdog_timer = self.create_timer(
            0.1,
            self.watchdog_callback
        )

        self.get_logger().info(
            '4WS ICR controller started '
            f'(L={self.wheelbase:.3f} m, '
            f'W={self.track_width:.3f} m, '
            f'Rwheel={self.wheel_radius:.3f} m)'
        )

    def clamp_steering(self, angle):
        return max(
            -self.max_steering_angle,
            min(self.max_steering_angle, angle)
        )

    def stop(self):
        steering_msg = Float64MultiArray()
        steering_msg.data = [0.0, 0.0, 0.0, 0.0]

        traction_msg = Float64MultiArray()
        traction_msg.data = [0.0, 0.0]

        self.steering_pub.publish(steering_msg)
        self.traction_pub.publish(traction_msg)

    def watchdog_callback(self):
        if self.last_cmd_vel_time is None:
            return

        elapsed = (
            self.get_clock().now() - self.last_cmd_vel_time
        ).nanoseconds / 1e9

        if elapsed >= self.cmd_vel_timeout and not self.watchdog_triggered:
            self.stop()
            self.watchdog_triggered = True

            self.get_logger().warning(
                f'cmd_vel timeout ({elapsed:.2f} s) - vehicle stopped'
            )

    def cmd_vel_callback(self, msg):

        self.last_cmd_vel_time = self.get_clock().now()
        self.watchdog_triggered = False

        v = float(msg.twist.linear.x)
        omega = float(msg.twist.angular.z)

        # ----------------------------------------------------------
        # STOP
        # Vehicle cannot perform zero-radius differential rotation.
        # ----------------------------------------------------------

        if abs(v) < 1e-4:
            self.stop()
            return

        # ----------------------------------------------------------
        # STRAIGHT MOTION
        # ----------------------------------------------------------

        if abs(omega) < 1e-4:

            wheel_speed = v / self.wheel_radius

            steering_msg = Float64MultiArray()
            steering_msg.data = [
                0.0,  # FL
                0.0,  # FR
                0.0,  # RL
                0.0   # RR
            ]

            traction_msg = Float64MultiArray()
            traction_msg.data = [
                wheel_speed,  # RL
                wheel_speed   # RR
            ]

            self.steering_pub.publish(steering_msg)
            self.traction_pub.publish(traction_msg)
            return

        # ----------------------------------------------------------
        # ICR-BASED SYMMETRIC COUNTER-PHASE 4WS
        #
        # Coordinate system:
        #
        #             +x
        #              ^
        #
        #       FL           FR
        #
        #              O  ------> +y left/right according to ROS
        #
        #       RL           RR
        #
        # For symmetric counter-phase 4WS, the vehicle centre has
        # zero lateral velocity.
        #
        # Desired body twist:
        #
        #   vx = v
        #   vy = 0
        #   wz = omega
        #
        # Therefore the ICR is:
        #
        #   R = v / omega
        #
        # measured laterally from the vehicle centre.
        #
        # Every wheel is then aligned with its own instantaneous
        # velocity vector:
        #
        #   vx_i = v - omega * y_i
        #   vy_i = omega * x_i
        #
        # steering_i = atan2(vy_i, vx_i)
        #
        # This gives different inner/outer steering angles while
        # keeping all four wheels consistent with the SAME rigid-body
        # motion / ICR.
        # ----------------------------------------------------------

        half_L = self.wheelbase / 2.0
        half_W = self.track_width / 2.0

        # Wheel positions relative to base centre.
        #
        # ROS convention:
        #   +x forward
        #   +y left
        #
        # FL: (+L/2, +W/2)
        # FR: (+L/2, -W/2)
        # RL: (-L/2, +W/2)
        # RR: (-L/2, -W/2)

        fl_vx = v - omega * half_W
        fl_vy = omega * half_L

        fr_vx = v + omega * half_W
        fr_vy = omega * half_L

        rl_vx = v - omega * half_W
        rl_vy = -omega * half_L

        rr_vx = v + omega * half_W
        rr_vy = -omega * half_L

        # Individual wheel steering directions.
        #
        # A wheel rolling line is equivalent modulo pi. For reverse motion,
        # atan2(vy, vx) may return an angle near +/-pi even though the
        # mechanically equivalent steering angle is near zero. Normalize
        # every steering command into [-pi/2, +pi/2] before applying the
        # physical steering limit.
        def normalize_steering(angle):
            while angle > math.pi / 2.0:
                angle -= math.pi
            while angle < -math.pi / 2.0:
                angle += math.pi
            return angle

        fl_angle = normalize_steering(math.atan2(fl_vy, fl_vx))
        fr_angle = normalize_steering(math.atan2(fr_vy, fr_vx))
        rl_angle = normalize_steering(math.atan2(rl_vy, rl_vx))
        rr_angle = normalize_steering(math.atan2(rr_vy, rr_vx))

        fl_angle = self.clamp_steering(fl_angle)
        fr_angle = self.clamp_steering(fr_angle)
        rl_angle = self.clamp_steering(rl_angle)
        rr_angle = self.clamp_steering(rr_angle)

        # ----------------------------------------------------------
        # REAR WHEEL SPEEDS
        #
        # Rear wheels are the driven wheels.
        #
        # Their required rolling speeds are the magnitudes of their
        # local rigid-body velocity vectors.
        #
        # Preserve the sign of commanded longitudinal motion so
        # reverse driving also works.
        # ----------------------------------------------------------

        direction = 1.0 if v >= 0.0 else -1.0

        rear_left_linear = direction * math.hypot(
            rl_vx,
            rl_vy
        )

        rear_right_linear = direction * math.hypot(
            rr_vx,
            rr_vy
        )

        rear_left_speed = (
            rear_left_linear / self.wheel_radius
        )

        rear_right_speed = (
            rear_right_linear / self.wheel_radius
        )

        # ----------------------------------------------------------
        # PUBLISH
        # ----------------------------------------------------------

        steering_msg = Float64MultiArray()
        steering_msg.data = [
            fl_angle,
            fr_angle,
            rl_angle,
            rr_angle
        ]

        traction_msg = Float64MultiArray()
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
