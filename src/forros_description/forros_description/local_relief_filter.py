#!/usr/bin/env python3
"""
Filters /camera/points down to only points that represent GENUINE
obstacles, using LOCAL RELIEF (height relative to neighboring terrain)
rather than absolute height off the ground.

See the original version's docstring for the full rationale (ramps vs
walls, why absolute-height thresholds fail on slopes). This version is
rewritten for real-time performance on CPU-only, embedded hardware
(target: Raspberry Pi 5, 16GB) - NO Python-level loops over points or
grid cells. Everything is vectorized numpy array operations.

Performance strategy for constrained hardware:
  1. Decimate the raw point cloud before any processing (a 640x480
     RGB-D cloud is ~300k points; we do not need anywhere near that
     many to detect terrain relief at a reasonable grid resolution).
  2. Vectorized binning using np.minimum.at (scatter-min) instead of a
     Python loop over every point.
  3. Vectorized neighbor comparison using array shifts (compare the
     whole height grid to itself, shifted by one cell in each
     direction) instead of a nested loop over every grid cell.
  4. Vectorized point classification using numpy fancy indexing
     instead of a loop over every point.
  5. Optional frame-skipping so the node only processes every Nth
     incoming cloud, trading update rate for headroom on weak CPUs.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def quat_to_matrix(x, y, z, w):
    """Quaternion -> 3x3 rotation matrix, no external deps."""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ])


class LocalReliefFilter(Node):
    def __init__(self):
        super().__init__('local_relief_filter')

        self.declare_parameter('input_topic', '/camera/points')
        self.declare_parameter('output_topic', '/camera/points_obstacles')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('grid_resolution', 0.10)
        self.declare_parameter('grid_half_extent', 1.5)
        self.declare_parameter('max_slope_deg', 25.0)
        self.declare_parameter('max_step_height', 0.06)
        self.declare_parameter('min_points_per_cell', 2)
        # --- Performance knobs for constrained hardware ---
        self.declare_parameter('decimation_stride', 8)    # keep 1-in-N raw points before any processing
        self.declare_parameter('process_every_n_frames', 2)  # skip frames to reduce load; 1 = process every frame

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.res = self.get_parameter('grid_resolution').value
        self.half_extent = self.get_parameter('grid_half_extent').value
        self.max_slope = math.radians(self.get_parameter('max_slope_deg').value)
        self.max_step = self.get_parameter('max_step_height').value
        self.min_pts = self.get_parameter('min_points_per_cell').value
        self.stride = max(1, int(self.get_parameter('decimation_stride').value))
        self.frame_skip = max(1, int(self.get_parameter('process_every_n_frames').value))

        self.grid_size = int(2 * self.half_extent / self.res) + 1
        self._frame_counter = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cloud_cb, 10)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.get_logger().info(
            f'local_relief_filter (vectorized) ready. {self.input_topic} -> {self.output_topic} | '
            f'grid {self.grid_size}x{self.grid_size} @ {self.res}m, '
            f'max_slope={math.degrees(self.max_slope):.1f}deg, max_step={self.max_step}m, '
            f'decimation=1/{self.stride}, processing 1/{self.frame_skip} frames')

    def cloud_cb(self, msg: PointCloud2):
        # --- Frame skipping: cheapest possible way to reduce load ---
        self._frame_counter += 1
        if self._frame_counter % self.frame_skip != 0:
            return

        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, msg.header.frame_id, Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed, skipping frame: {e}', throttle_duration_sec=2.0)
            return

        # --- Decode once as a structured array, then decimate immediately ---
        pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        if pts.shape[0] == 0:
            return
        # skip_nans only removes NaN, NOT Infinity - this camera's raw
        # output includes Infinity for out-of-range pixels (confirmed
        # earlier), so we must explicitly filter those too, or a single
        # leaked Infinity point can corrupt an entire grid cell's min-height
        # via np.minimum.at, producing a false "infinite step" obstacle.
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        if pts.shape[0] == 0:
            return
        pts = pts[::self.stride]
        if pts.shape[0] == 0:
            return

        # --- Vectorized transform into base_frame ---
        tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        qx, qy, qz, qw = (t.transform.rotation.x, t.transform.rotation.y,
                           t.transform.rotation.z, t.transform.rotation.w)
        R = quat_to_matrix(qx, qy, qz, qw)
        transformed = pts @ R.T + np.array([tx, ty, tz])

        gx = np.floor((transformed[:, 0] + self.half_extent) / self.res).astype(np.int32)
        gy = np.floor((transformed[:, 1] + self.half_extent) / self.res).astype(np.int32)
        gz = transformed[:, 2]

        valid = (gx >= 0) & (gx < self.grid_size) & (gy >= 0) & (gy < self.grid_size)
        if not np.any(valid):
            return
        gx, gy, gz = gx[valid], gy[valid], gz[valid]
        transformed = transformed[valid]

        # --- Vectorized per-cell minimum height via scatter-min ---
        cell_id = gx.astype(np.int64) * self.grid_size + gy.astype(np.int64)
        n_cells = self.grid_size * self.grid_size
        min_height_flat = np.full(n_cells, np.inf, dtype=np.float64)
        np.minimum.at(min_height_flat, cell_id, gz)
        counts_flat = np.bincount(cell_id, minlength=n_cells)

        min_height = min_height_flat.reshape(self.grid_size, self.grid_size)
        counts = counts_flat.reshape(self.grid_size, self.grid_size)
        valid_cell = counts >= self.min_pts
        min_height[~valid_cell] = np.nan

        # --- Vectorized neighbor comparison via array shifts (no cell loop) ---
        # NOTE: np.roll wraps around at grid edges - without correction, the
        # far edge of the grid would be falsely compared against the near
        # edge as if they were adjacent. We explicitly invalidate the
        # wrapped-around border after each shift so edge cells only ever
        # compare against real neighbors (or nothing, if at the true edge).
        obstacle_cell = np.zeros((self.grid_size, self.grid_size), dtype=bool)
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            shifted = np.roll(min_height, shift=(dx, dy), axis=(0, 1))
            shifted_valid = np.roll(valid_cell, shift=(dx, dy), axis=(0, 1))
            if dx == 1:
                shifted_valid[0, :] = False
            elif dx == -1:
                shifted_valid[-1, :] = False
            if dy == 1:
                shifted_valid[:, 0] = False
            elif dy == -1:
                shifted_valid[:, -1] = False
            with np.errstate(invalid='ignore'):
                step = np.abs(min_height - shifted)
                slope = np.arctan2(step, self.res)
                flagged = (step > self.max_step) | (slope > self.max_slope)
            flagged = flagged & valid_cell & shifted_valid
            obstacle_cell |= np.nan_to_num(flagged, nan=False).astype(bool)

        # --- Vectorized point classification (fancy indexing, no loop) ---
        keep_mask = obstacle_cell[gx, gy]
        obstacle_points = transformed[keep_mask]

        if obstacle_points.shape[0] == 0:
            return

        out_header = msg.header
        out_header.frame_id = self.base_frame
        out_msg = pc2.create_cloud(
            header=out_header,
            fields=[
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ],
            points=obstacle_points.astype(np.float32),
        )
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalReliefFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()