#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class FourWSOdometry(Node):

    def __init__(self):
        super().__init__('four_ws_odometry')

        # Vehicle geometry
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

        # Integrated odometry state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = None

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            20
        )

        self.get_logger().info(
            '4WS odometry started '
            f'(L={self.wheelbase:.3f} m, '
            f'W={self.track_width:.3f} m, '
            f'R={self.wheel_radius:.3f} m)'
        )

    def joint_state_callback(self, msg):

        joint_index = {
            name: i for i, name in enumerate(msg.name)
        }

        required_joints = [
            'front_left_steering_joint',
            'front_right_steering_joint',
            'rear_left_steering_joint',
            'rear_right_steering_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]

        if not all(name in joint_index for name in required_joints):
            return

        # ----------------------------------------------------------
        # READ MEASURED STEERING POSITIONS
        # ----------------------------------------------------------

        fl_angle = msg.position[
            joint_index['front_left_steering_joint']
        ]

        fr_angle = msg.position[
            joint_index['front_right_steering_joint']
        ]

        rl_angle = msg.position[
            joint_index['rear_left_steering_joint']
        ]

        rr_angle = msg.position[
            joint_index['rear_right_steering_joint']
        ]

        # One physical steering position per axle.
        front_angle = 0.5 * (fl_angle + fr_angle)
        rear_angle = 0.5 * (rl_angle + rr_angle)

        # ----------------------------------------------------------
        # READ MEASURED REAR WHEEL VELOCITIES
        # ----------------------------------------------------------

        if len(msg.velocity) <= max(
            joint_index['rear_left_wheel_joint'],
            joint_index['rear_right_wheel_joint']
        ):
            return

        rear_left_angular = msg.velocity[
            joint_index['rear_left_wheel_joint']
        ]

        rear_right_angular = msg.velocity[
            joint_index['rear_right_wheel_joint']
        ]

        rear_left_linear = (
            rear_left_angular * self.wheel_radius
        )

        rear_right_linear = (
            rear_right_angular * self.wheel_radius
        )

        # Vehicle longitudinal velocity estimated from the
        # two driven rear wheels.
        linear_velocity = 0.5 * (
            rear_left_linear + rear_right_linear
        )

        # ----------------------------------------------------------
        # 4WS YAW-RATE ESTIMATE
        #
        # General bicycle-model relationship:
        #
        # yaw_rate =
        #     v / L * (tan(front) - tan(rear))
        #
        # For symmetric counter-phase steering:
        #
        # rear = -front
        #
        # which becomes:
        #
        # yaw_rate = 2*v/L * tan(front)
        # ----------------------------------------------------------

        angular_velocity = (
            linear_velocity / self.wheelbase
        ) * (
            math.tan(front_angle) -
            math.tan(rear_angle)
        )

        # ----------------------------------------------------------
        # TIME STEP
        # Use ROS simulation time.
        # ----------------------------------------------------------

        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (
            current_time - self.last_time
        ).nanoseconds / 1e9

        self.last_time = current_time

        if dt <= 0.0 or dt > 0.5:
            return

        # ----------------------------------------------------------
        # INTEGRATE VEHICLE POSE
        #
        # Midpoint integration gives better accuracy during curves
        # than integrating translation using the old yaw directly.
        # ----------------------------------------------------------

        delta_yaw = angular_velocity * dt
        yaw_mid = self.yaw + 0.5 * delta_yaw

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

        half_yaw = self.yaw * 0.5

        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        # ----------------------------------------------------------
        # nav_msgs/Odometry
        # ----------------------------------------------------------

        odom = Odometry()

        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom)

        # ----------------------------------------------------------
        # TF: odom -> base_footprint
        # ----------------------------------------------------------

        transform = TransformStamped()

        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)


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
