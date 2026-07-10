#!/usr/bin/env python3
"""
Filters the HORIZONTAL lidar's /scan using traversability info from the
TILTED lidar, so that ramps/slopes correctly classified as traversable
don't get treated as walls by Nav2's obstacle layer.

Why this is needed:
  The horizontal lidar only sees a single flat height slice. When a ramp
  rises up to intersect that slice, the horizontal lidar reports a hit
  there - indistinguishable from a real wall. Meanwhile the tilted lidar
  (via traversability_check.py) knows that same patch of ground is
  actually a climbable slope, not an obstacle.

  This node cross-references the two: for every tilted-lidar beam
  classified as flat/traversable-slope, it computes which HORIZONTAL
  beam angle that same patch of ground corresponds to (using the known
  fixed mounting rotation between the two sensors), and blanks out
  (sets to inf) that beam in a filtered copy of /scan before Nav2 ever
  sees it.

  Beams the horizontal lidar sees that the tilted lidar did NOT confirm
  as traversable are passed through unchanged - so real walls/obstacles
  still register normally.

APPROXIMATION NOTE:
  This ignores the small (~10cm) translation offset between the two
  lidar mounting positions and only accounts for the rotational
  (pitch) offset between them. This is a reasonable approximation once
  a hit is more than ~1m away, but will be less accurate for very
  close-range ramps/steps. Good enough as a first pass; revisit if you
  see filtering misbehave at close range.

Publishes:
  /scan_filtered (sensor_msgs/LaserScan) - same as /scan, but with
  confirmed-traversable-ramp beams blanked out. Point Nav2's
  ObstacleLayer at THIS topic instead of raw /scan.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')

        # Must match the tilted lidar's mount pitch used in
        # traversability_check.py (magnitude only; direction doesn't
        # matter for this azimuth-projection math).
        self.declare_parameter('tilt_mount_pitch', 0.6109)
        # How many extra horizontal beams on either side of the computed
        # match to also blank out, to account for the translation
        # approximation and beam-width mismatch between the two sensors.
        self.declare_parameter('blank_window', 2)

        self.pitch = abs(self.get_parameter('tilt_mount_pitch').value)
        self.blank_window = self.get_parameter('blank_window').value

        # Cache of latest tilted-lidar geometry + classification
        self._tilt_angle_min = None
        self._tilt_angle_increment = None
        self._tilt_classes = None

        self.create_subscription(LaserScan, '/scan_tilt', self.tilt_scan_cb, 10)
        self.create_subscription(Float32MultiArray, '/traversability_classes', self.classes_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.horizontal_scan_cb, 10)

        self.filtered_pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

        self.get_logger().info('scan_filter ready. Publishing /scan_filtered.')

    def tilt_scan_cb(self, msg: LaserScan):
        self._tilt_angle_min = msg.angle_min
        self._tilt_angle_increment = msg.angle_increment

    def classes_cb(self, msg: Float32MultiArray):
        self._tilt_classes = msg.data

    def horizontal_scan_cb(self, msg: LaserScan):
        if self._tilt_classes is None or self._tilt_angle_min is None:
            # No traversability data yet - pass through unfiltered.
            self.filtered_pub.publish(msg)
            return

        ranges = list(msg.ranges)
        n_horizontal = len(ranges)

        for i, cls in enumerate(self._tilt_classes):
            if cls not in (0.0, 2.0):  # 0.0=flat, 2.0=traversable slope
                continue

            tilt_angle = self._tilt_angle_min + i * self._tilt_angle_increment

            # Project this tilted-lidar beam direction into base_link
            # frame (rotation about Y by mount pitch only), then find
            # its azimuth (angle about Z) - this tells us which
            # direction on the HORIZONTAL lidar's own flat scan plane
            # this same patch of ground corresponds to.
            cx = math.cos(tilt_angle)
            sx = math.sin(tilt_angle)
            x_proj = math.cos(self.pitch) * cx
            y_proj = sx
            azimuth = math.atan2(y_proj, x_proj)

            # Map that azimuth to a horizontal-scan beam index.
            idx = round((azimuth - msg.angle_min) / msg.angle_increment)

            for j in range(idx - self.blank_window, idx + self.blank_window + 1):
                if 0 <= j < n_horizontal:
                    ranges[j] = float('inf')

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = ranges
        out.intensities = msg.intensities

        self.filtered_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
