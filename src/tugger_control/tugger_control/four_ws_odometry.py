#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


class FourWSOdometry(Node):

    def __init__(self):
        super().__init__('four_ws_odometry')

        self.declare_parameter('wheelbase', 0.58)
        self.declare_parameter('track_width', 0.44)
        self.declare_parameter('wheel_radius', 0.055)

        self.wheelbase = float(
            self.get_parameter('wheelbase').value
        )

        self.track_width = float(
            self.get_parameter('track_width').value
        )

        self.wheel_radius = float(
            self.get_parameter('wheel_radius').value
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = None

        self.odom_pub = self.create_publisher(
            Odometry,
            '/wheel/odom',
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            20
        )

        self.get_logger().info(
            '4WS ICR wheel odometry started '
            f'(L={self.wheelbase:.3f} m, '
            f'W={self.track_width:.3f} m, '
            f'R={self.wheel_radius:.3f} m)'
        )

    def joint_state_callback(self, msg):

        joint_index = {
            name: i for i, name in enumerate(msg.name)
        }

        required_joints = [
            'rear_left_steering_joint',
            'rear_right_steering_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]

        if not all(
            name in joint_index
            for name in required_joints
        ):
            return

        rl_steer_index = joint_index[
            'rear_left_steering_joint'
        ]

        rr_steer_index = joint_index[
            'rear_right_steering_joint'
        ]

        rl_wheel_index = joint_index[
            'rear_left_wheel_joint'
        ]

        rr_wheel_index = joint_index[
            'rear_right_wheel_joint'
        ]

        if len(msg.position) <= max(
            rl_steer_index,
            rr_steer_index
        ):
            return

        if len(msg.velocity) <= max(
            rl_wheel_index,
            rr_wheel_index
        ):
            return

        # Actual rear steering angles.
        delta_rl = msg.position[rl_steer_index]
        delta_rr = msg.position[rr_steer_index]

        # Actual rear wheel angular velocities.
        omega_rl_wheel = msg.velocity[rl_wheel_index]
        omega_rr_wheel = msg.velocity[rr_wheel_index]

        # Signed rolling speeds at each rear wheel.
        speed_rl = (
            omega_rl_wheel *
            self.wheel_radius
        )

        speed_rr = (
            omega_rr_wheel *
            self.wheel_radius
        )

        # ----------------------------------------------------
        # Reconstruct body twist from rear-wheel kinematics.
        #
        # Wheel velocity at body position (x_i, y_i):
        #
        #   vx_i = vx - wz * y_i
        #   vy_i =      wz * x_i
        #
        # A rolling wheel with steering angle delta_i has:
        #
        #   vx_i = s_i * cos(delta_i)
        #   vy_i = s_i * sin(delta_i)
        #
        # Rear axle:
        #
        #   x_i = -L / 2
        #
        # ----------------------------------------------------

        rear_x = -0.5 * self.wheelbase

        # First estimate yaw rate from the lateral component.
        wz_rl = (
            speed_rl *
            math.sin(delta_rl) /
            rear_x
        )

        wz_rr = (
            speed_rr *
            math.sin(delta_rr) /
            rear_x
        )

        angular_velocity = 0.5 * (
            wz_rl + wz_rr
        )

        # Correct longitudinal estimates for each wheel's
        # lateral position:
        #
        # vx_i = vx - wz*y_i
        # therefore:
        # vx = vx_i + wz*y_i
        #
        # RL: y = +W/2
        # RR: y = -W/2

        vx_from_rl = (
            speed_rl * math.cos(delta_rl)
            + angular_velocity *
            (0.5 * self.track_width)
        )

        vx_from_rr = (
            speed_rr * math.cos(delta_rr)
            - angular_velocity *
            (0.5 * self.track_width)
        )

        linear_velocity = 0.5 * (
            vx_from_rl +
            vx_from_rr
        )

        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (
            current_time -
            self.last_time
        ).nanoseconds / 1e9

        self.last_time = current_time

        if dt <= 0.0 or dt > 0.5:
            return

        # Midpoint integration.
        delta_yaw = (
            angular_velocity *
            dt
        )

        yaw_mid = (
            self.yaw +
            0.5 * delta_yaw
        )

        self.x += (
            linear_velocity *
            math.cos(yaw_mid) *
            dt
        )

        self.y += (
            linear_velocity *
            math.sin(yaw_mid) *
            dt
        )

        self.yaw += delta_yaw

        self.yaw = math.atan2(
            math.sin(self.yaw),
            math.cos(self.yaw)
        )

        self.publish_odometry(
            current_time,
            linear_velocity,
            angular_velocity
        )

    def publish_odometry(
        self,
        current_time,
        linear_velocity,
        angular_velocity
    ):

        half_yaw = (
            0.5 * self.yaw
        )

        odom = Odometry()

        odom.header.stamp = (
            current_time.to_msg()
        )

        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.z = (
            math.sin(half_yaw)
        )

        odom.pose.pose.orientation.w = (
            math.cos(half_yaw)
        )

        odom.twist.twist.linear.x = (
            linear_velocity
        )

        odom.twist.twist.linear.y = 0.0

        odom.twist.twist.angular.z = (
            angular_velocity
        )

        # Basic non-zero covariance for EKF input.
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05

        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.03

        self.odom_pub.publish(odom)


def main(args=None):

    rclpy.init(args=args)

    node = FourWSOdometry()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
