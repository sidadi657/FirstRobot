#!/usr/bin/env python3
"""
Traversability estimation from a single tilted 2D lidar.

How it works:
  The tilted lidar is mounted at a known height above the ground and
  pitched down by a known angle. If the ground ahead were perfectly
  flat, every beam would return a predictable range, computable from
  simple trigonometry. By comparing the ACTUAL measured range at each
  beam angle to this PREDICTED flat-ground range, we can classify what
  the beam is actually seeing:

    - actual ~= predicted        -> flat, safe ground
    - actual <  predicted        -> something sticking up (obstacle)
    - actual >  predicted (finite) -> ground sloping away (ramp/dip);
                                       we back out the real slope angle
    - actual == inf               -> drop-off / cliff / nothing there

Publishes a MarkerArray for RViz visualization (green/yellow/red dots)
and a Float32MultiArray with a numeric class per beam:
    0.0 = safe/flat
    1.0 = obstacle
    2.0 = traversable slope (angle within limit)
    3.0 = too steep (exceeds limit)
    4.0 = dropoff / unknown
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class TraversabilityCheck(Node):
    def __init__(self):
        super().__init__('traversability_check')

        # --- Mount geometry: EDIT THESE to match your r-eal robot ---
        self.declare_parameter('mount_height', 0.12)      # meters, lidar height above GROUND (not base_link!)
        self.declare_parameter('mount_pitch', 0.6109)      # radians, magnitude of downward tilt (~35 deg)
        self.declare_parameter('pitch_points_down', True)  # set False if your tilt is actually upward

        # --- Traversal limits: set from your rocker-bogie's real specs ---
        self.declare_parameter('max_slope_deg', 25.0)      # max climbable slope angle
        self.declare_parameter('obstacle_margin', 0.05)    # meters; range must be this much SHORTER than predicted to count as obstacle
        self.declare_parameter('slope_margin', 0.05)       # meters; range must be this much LONGER than predicted to count as a slope (not just noise)

        # --- Self-filter: beams hitting the robot's own chassis ---
        self.declare_parameter('self_filter_min_range', 0.15)  # meters; any return closer than this is assumed to be the robot's own body, not terrain
        self.declare_parameter('exclude_angle_min', -1.57)   # radians; band of angles to BLANK OUT (e.g. where chassis sits). Set exclude_angle_min == exclude_angle_max to disable.
        self.declare_parameter('exclude_angle_max', 1.57)

        self.h = self.get_parameter('mount_height').value
        self.pitch = abs(self.get_parameter('mount_pitch').value)
        self.points_down = self.get_parameter('pitch_points_down').value
        self.max_slope = math.radians(self.get_parameter('max_slope_deg').value)
        self.obstacle_margin = self.get_parameter('obstacle_margin').value
        self.slope_margin = self.get_parameter('slope_margin').value
        self.self_filter_min_range = self.get_parameter('self_filter_min_range').value
        self.exclude_angle_min = self.get_parameter('exclude_angle_min').value
        self.exclude_angle_max = self.get_parameter('exclude_angle_max').value

        self.sub = self.create_subscription(LaserScan, '/scan_tilt', self.scan_cb, 10)
        self.class_pub = self.create_publisher(Float32MultiArray, '/traversability_classes', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/traversability_markers', 10)

        if not self.points_down:
            self.get_logger().warn(
                'pitch_points_down is False - this node assumes the tilted '
                'lidar looks down at the ground ahead. If it is actually '
                'looking up/away from the ground, this math will not produce '
                'meaningful results.')

        self.get_logger().info(
            f'traversability_check ready. mount_height={self.h}m, '
            f'pitch={math.degrees(self.pitch):.1f}deg, '
            f'max_slope={math.degrees(self.max_slope):.1f}deg')

    def scan_cb(self, msg: LaserScan):
        classes = []
        markers = MarkerArray()

        # Clear previous markers
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, r in enumerate(msg.ranges):
            beam_angle = msg.angle_min + i * msg.angle_increment

            # --- Self-filter: drop beams pointed at a known chassis blind
            # spot (a specific band), and drop close-range hits anywhere ---
            if self.exclude_angle_min != self.exclude_angle_max and \
               self.exclude_angle_min <= beam_angle <= self.exclude_angle_max:
                classes.append(4.0)
                continue
            if not (math.isinf(r) or math.isnan(r)) and r < self.self_filter_min_range:
                classes.append(4.0)
                continue

            # Effective downward angle of THIS beam = mount pitch + beam's
            # own angle within the scan plane. Only beams pointing enough
            # "downward" will ever intersect flat ground at a finite range.
            effective_angle = self.pitch + beam_angle

            if effective_angle <= 0.01 or effective_angle >= (math.pi - 0.01):
                # This beam points above the horizon or straight along it -
                # no flat-ground intersection is geometrically meaningful.
                classes.append(4.0)
                continue

            predicted = self.h / math.sin(effective_angle)

            if math.isinf(r) or math.isnan(r) or r > msg.range_max:
                cls = 4.0  # dropoff / nothing there
            elif r < predicted - self.obstacle_margin:
                cls = 1.0  # obstacle sticking up
            elif r > predicted + self.slope_margin:
                # Ground slopes away here - back out the real slope angle.
                # Predicted assumed flat ground at 'predicted' distance;
                # actual hit is further out at range r. Use the geometry
                # difference to estimate local slope.
                # Height where beam WOULD have hit if flat:
                flat_drop = self.h
                # Height actually reached at range r along the same beam:
                actual_drop = r * math.sin(effective_angle)
                # Horizontal distances
                flat_horiz = predicted * math.cos(effective_angle)
                actual_horiz = r * math.cos(effective_angle)
                horiz_delta = max(actual_horiz - flat_horiz, 1e-3)
                drop_delta = flat_drop - actual_drop  # positive = ground fell away
                slope_angle = math.atan2(abs(drop_delta), horiz_delta)

                if slope_angle <= self.max_slope:
                    cls = 2.0  # traversable slope
                else:
                    cls = 3.0  # too steep
            else:
                cls = 0.0  # flat, safe

            classes.append(cls)

            # Skip marker creation for invalid ranges (inf/nan) - nothing
            # sensible to draw, and RViz will reject the whole MarkerArray
            # if any single marker has non-finite coordinates.
            if math.isinf(r) or math.isnan(r):
                continue

            # Build a marker for visualization
            x = r * math.cos(beam_angle)
            y = r * math.sin(beam_angle)
            m = Marker()
            m.header = msg.header
            m.ns = 'traversability'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.a = 1.0
            if cls == 0.0:
                m.color.g = 1.0
            elif cls == 1.0:
                m.color.r = 1.0
            elif cls == 2.0:
                m.color.r = 1.0
                m.color.g = 1.0
            elif cls == 3.0:
                m.color.r = 1.0
                m.color.g = 0.5
            else:
                m.color.b = 1.0
            markers.markers.append(m)

        out = Float32MultiArray()
        out.data = classes
        self.class_pub.publish(out)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = TraversabilityCheck()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()