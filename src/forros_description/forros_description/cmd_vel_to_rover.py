#!/usr/bin/env python3
"""
Converts geometry_msgs/Twist on /cmd_vel into commands for a 6-wheel
rocker-bogie rover: front-left, front-right, mid-left, mid-right,
back-left, back-right. Only the FRONT and BACK corners have steering
actuators - the MID wheels have no steering joint, they only drive.

Steering angle is a DIRECT function of angular.z (not v/omega), so it's
continuous and won't snap to full lock on a tiny stick nudge at v=0.

Publishes:
  - /steering_controller/commands        (std_msgs/Float64MultiArray) - 4 values (front_left, front_right, back_left, back_right)
  - /rover_velocity_controller/commands  (std_msgs/Float64MultiArray) - 6 values (all wheels)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class CmdVelToRover(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_rover')

        # --- Tuning ---
        self.declare_parameter('velocity_scale', 5.0)   # multiplies linear.x
        self.declare_parameter('steer_gain', 5.0)         # rad of steer per rad/s of angular.z, tune this for sensitivity
        self.declare_parameter('rear_steer_factor', -1.0) # -1.0 = rear steers opposite front (tight pivot), 0.0 = rear fixed straight (car-like), 1.0 = rear same as front (crab/sideways)
        self.declare_parameter('diff_gain', 0.5)          # how much left/right wheel speed splits during a turn
        self.declare_parameter('track_width', 0.25920)    # left wheel to right wheel (m), used only for diff_gain speed split
        self.declare_parameter('max_steer_angle', 1.5708) # rad, matches your ros2_control limits
        self.declare_parameter('turn_speed_scale', 10.0)  # scales omega's contribution to wheel speed, independent of linear velocity_scale

        self.v_scale = self.get_parameter('velocity_scale').value
        self.steer_gain = self.get_parameter('steer_gain').value
        self.rear_factor = self.get_parameter('rear_steer_factor').value
        self.diff_gain = self.get_parameter('diff_gain').value
        self.W = self.get_parameter('track_width').value
        self.max_steer = self.get_parameter('max_steer_angle').value
        self.turn_scale = self.get_parameter('turn_speed_scale').value
        # Only 4 corners have a steering actuator - mid wheels are drive-only.
        # Order MUST match ros2_controllers.yaml for steering_controller.
        self.steer_order = ['front_left', 'front_right', 'back_left', 'back_right']

        # All 6 wheels get a drive command.
        # Order MUST match ros2_controllers.yaml for rover_velocity_controller.
        self.drive_order = ['back_left', 'front_left', 'back_right', 'front_right', 'mid_left', 'mid_right']

        # --- Mirrored-joint fix ---
        # Left side as-is (+1), right side flipped (-1) - classic
        # mirrored-axis symptom from the Fusion export. Added mid_left/
        # mid_right here too since the same export issue usually hits all
        # three axles identically. If your mid wheels turn out NOT to be
        # mirrored, just set mid_right back to 1.0.
        self.wheel_drive_sign = {
            'front_left': 1.0, 'front_right': 1.0,
            'mid_left': 1.0, 'mid_right': 1.0,
            'back_left': 1.0, 'back_right': 1.0,
        }
        # Steer sign only applies to the 4 steered corners.
        self.wheel_steer_sign = {
            'front_left': 1.0, 'front_right': -1.0,
            'back_left': 1.0, 'back_right': -1.0,
        }

        self.steer_pub = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/rover_velocity_controller/commands', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.get_logger().info('cmd_vel_to_rover (6-wheel rocker-bogie) ready. Listening on /cmd_vel')

    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x * self.v_scale
        omega = msg.angular.z

        # --- Steering: direct proportional mapping, no division ---
        # front angle scales with omega; rear angle scales with omega * rear_factor
        # mid wheels have no entry here at all - they physically can't steer.
        front_angle = self._clamp(self.steer_gain * omega, self.max_steer)
        rear_angle = self._clamp(self.steer_gain * omega * self.rear_factor, self.max_steer)

        corner_angle = {
            'front_left': front_angle, 'front_right': front_angle,
            'back_left': rear_angle, 'back_right': rear_angle,
        }

        steer_msg = Float64MultiArray()
        steer_msg.data = [
            corner_angle[name] * self.wheel_steer_sign[name]
            for name in self.steer_order
        ]
        self.steer_pub.publish(steer_msg)

        # --- Drive speed: simple differential split, no division ---
        # Same left/right speed split applied across all three axles
        # (front, mid, back). This is a skid-steer style approximation -
        # it doesn't compute a true per-wheel instantaneous turn radius,
        # but it matches the level of simplification the original script
        # used and works fine in practice for rocker-bogie platforms.
        half_track = self.W / 2.0
        left_speed = v - omega * half_track * self.diff_gain * self.turn_scale
        right_speed = v + omega * half_track * self.diff_gain* self.turn_scale

        wheel_speed = {
            'front_left': left_speed, 'mid_left': left_speed, 'back_left': left_speed,
            'front_right': right_speed, 'mid_right': right_speed, 'back_right': right_speed,
        }

        vel_msg = Float64MultiArray()
        vel_msg.data = [
            wheel_speed[name] * self.wheel_drive_sign[name]
            for name in self.drive_order
        ]
        self.vel_pub.publish(vel_msg)

    def _clamp(self, val, limit):
        return max(-limit, min(limit, val))


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()