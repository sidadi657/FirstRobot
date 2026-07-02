#!/usr/bin/env python3
"""
Converts geometry_msgs/Twist on /cmd_vel into the two command topics
your ForwardCommandControllers expect:
  - /steering_controller/commands        (std_msgs/Float64MultiArray)
  - /rover_velocity_controller/commands  (std_msgs/Float64MultiArray)

Uses an instantaneous-center-of-rotation model. Front/back corners steer,
mid wheels stay fixed at 0 (like Curiosity/Perseverance).

IMPORTANT: set wheel_base_front, wheel_base_rear and track_width to your
actual robot's geometry (measure from your xacro joint origins - distance
along X from the mid-wheel axle to the front/back axle, and full track
width across Y). Values below are placeholders.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class CmdVelToRover(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_rover')

        # --- Robot geometry (EDIT THESE to match your xacro) ---
        self.declare_parameter('wheel_base_front', 0.30)  # mid-wheel axle -> front axle (m)
        self.declare_parameter('wheel_base_rear', 0.30)   # mid-wheel axle -> rear axle (m)
        self.declare_parameter('track_width', 0.40)        # left wheel to right wheel (m)
        self.declare_parameter('max_steer_angle', 1.5708)  # rad, matches your ros2_control limits
        self.declare_parameter('straight_omega_threshold', 0.02)

        self.Lf = self.get_parameter('wheel_base_front').value
        self.Lr = self.get_parameter('wheel_base_rear').value
        self.W = self.get_parameter('track_width').value
        self.max_steer = self.get_parameter('max_steer_angle').value
        self.omega_eps = self.get_parameter('straight_omega_threshold').value

        # Corner positions in robot frame (x forward, y left), origin at mid-wheel axle
        self.corners = {
            'front_left':  ( self.Lf,  self.W / 2.0),
            'front_right': ( self.Lf, -self.W / 2.0),
            'back_left':   (-self.Lr,  self.W / 2.0),
            'back_right':  (-self.Lr, -self.W / 2.0),
        }

        self.steer_pub = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/rover_velocity_controller/commands', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.get_logger().info('cmd_vel_to_rover ready. Listening on /cmd_vel')

    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z

        steer_angles = []  # order: front_left, front_right, back_left, back_right
        # matches ros2_controllers.yaml order:
        # motor_house_left_front, motor_house_right_front, motor_house_left_back, motor_house_right_back
        corner_order = ['front_left', 'front_right', 'back_left', 'back_right']

        if abs(omega) < self.omega_eps:
            # Straight line: no steering, all corners point forward
            for _ in corner_order:
                steer_angles.append(0.0)
            wheel_speeds = self.straight_wheel_speeds(v)
        else:
            R = v / omega  # signed turning radius, +y (left) convention
            for name in corner_order:
                x_w, y_w = self.corners[name]
                angle = math.atan2(x_w, R - y_w)
                angle = max(-self.max_steer, min(self.max_steer, angle))
                steer_angles.append(angle)
            wheel_speeds = self.turning_wheel_speeds(omega, R)

        steer_msg = Float64MultiArray()
        steer_msg.data = steer_angles
        self.steer_pub.publish(steer_msg)

        vel_msg = Float64MultiArray()
        vel_msg.data = wheel_speeds
        self.vel_pub.publish(vel_msg)

    def straight_wheel_speeds(self, v):
        # order in ros2_controllers.yaml:
        # left_back_wheel, left_front_wheel, right_back_wheel,
        # right_front_wheel, left_mid_wheel, right_mid_wheel
        return [v, v, v, v, v, v]

    def turning_wheel_speeds(self, omega, R):
        wheel_positions = {
            'left_back_wheel':   (-self.Lr,  self.W / 2.0),
            'left_front_wheel':  ( self.Lf,  self.W / 2.0),
            'right_back_wheel':  (-self.Lr, -self.W / 2.0),
            'right_front_wheel': ( self.Lf, -self.W / 2.0),
            'left_mid_wheel':    (0.0,  self.W / 2.0),
            'right_mid_wheel':   (0.0, -self.W / 2.0),
        }
        order = ['left_back_wheel', 'left_front_wheel', 'right_back_wheel',
                 'right_front_wheel', 'left_mid_wheel', 'right_mid_wheel']

        speeds = []
        for name in order:
            x_w, y_w = wheel_positions[name]
            dist = math.hypot(x_w, y_w - R)
            speeds.append(omega * dist)
        return speeds


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()