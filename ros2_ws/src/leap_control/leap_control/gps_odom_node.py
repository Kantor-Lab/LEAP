#!/usr/bin/env python3
"""
gps_odom_node.py

Converts RTK NavSatFix messages (e.g. from nmea_navsat_driver on a Reach M2)
into nav_msgs/Odometry in the robot's local/map frame, using a precomputed
4x4 rigid transform T_world_utm such that:

    p_local = T_world_utm @ p_utm

The transform file is expected in the format produced by the mapping
pipeline: a 4x4 matrix (whitespace-delimited, 4 rows) followed by comment
lines starting with '#', one of which specifies the UTM zone, e.g.:

    # UTM zone: 17N

This node does NOT use navsat_transform_node's auto-datum estimation.
It assumes the local frame this transform targets IS the robot's live
map/world origin (per LEAP convention) -- if that ever changes, this
transform must be regenerated.
"""

import re

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry

try:
    from pyproj import Transformer
except ImportError as e:
    raise ImportError(
        "pyproj is required for gps_odom_node. Install with: "
        "pip install pyproj --break-system-packages (or add to your rosdep/venv)."
    ) from e


def load_transform(path: str):
    """Load the 4x4 T_world_utm matrix and UTM zone from a text file."""
    matrix_rows = []
    zone_number = None
    zone_hemisphere = None

    with open(path, 'r') as f:
        for line in f:
            stripped = line.strip()
            if not stripped:
                continue
            if stripped.startswith('#'):
                m = re.search(r'UTM zone:\s*(\d+)\s*([NnSs])', stripped)
                if m:
                    zone_number = int(m.group(1))
                    zone_hemisphere = m.group(2).upper()
                continue
            matrix_rows.append([float(x) for x in stripped.split()])

    if zone_number is None:
        raise ValueError(
            f"Could not find a '# UTM zone: <N><hemisphere>' comment line in {path}"
        )

    T = np.array(matrix_rows, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"Matrix in {path} is not 4x4, got shape {T.shape}")

    return T, zone_number, zone_hemisphere


class GpsOdomNode(Node):
    def __init__(self):
        super().__init__('gps_odom_node')

        # --- Parameters ---
        self.declare_parameter('transform_file', '')
        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('odom_topic', '/odometry/gps')
        self.declare_parameter('child_frame_id', 'base_footprint')
        self.declare_parameter('geoid_offset', 0.0)  # meters, see note below
        self.declare_parameter('min_publish_status', NavSatStatus.STATUS_FIX)
        self.declare_parameter('default_position_stddev', 0.02)  # meters, fallback only

        transform_file = self.get_parameter('transform_file').get_parameter_value().string_value
        if not transform_file:
            raise RuntimeError('Parameter "transform_file" must be set to the path of T_world_utm.txt')

        self.gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_frame_id = "map"
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.geoid_offset = self.get_parameter('geoid_offset').get_parameter_value().double_value
        self.min_publish_status = self.get_parameter('min_publish_status').get_parameter_value().integer_value
        self.default_position_stddev = self.get_parameter('default_position_stddev').get_parameter_value().double_value

        # --- Load transform ---
        self.T, zone_number, hemisphere = load_transform(transform_file)
        self.R = self.T[:3, :3]
        self.t = self.T[:3, 3]

        epsg = 32600 + zone_number if hemisphere == 'N' else 32700 + zone_number
        self.get_logger().info(
            f'Loaded T_world_utm from {transform_file} (UTM zone {zone_number}{hemisphere}, EPSG:{epsg})'
        )

        self.to_utm = Transformer.from_crs('EPSG:4326', f'EPSG:{epsg}', always_xy=True)

        # --- Pub/Sub ---
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(NavSatFix, self.gps_topic, self.gps_callback, qos)
        self.pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self._warned_no_fix = False

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < self.min_publish_status:
            if not self._warned_no_fix:
                self.get_logger().warn(
                    f'GPS status {msg.status.status} below min_publish_status '
                    f'{self.min_publish_status}; suppressing further warnings until fix recovers.'
                )
                self._warned_no_fix = True
            return
        self._warned_no_fix = False

        if msg.latitude == 0.0 and msg.longitude == 0.0:
            self.get_logger().warn('Received (0,0) lat/lon, skipping.')
            return

        easting, northing = self.to_utm.transform(msg.longitude, msg.latitude)
        altitude = msg.altitude + self.geoid_offset

        p_utm = np.array([easting, northing, altitude])
        p_local = self.R @ p_utm + self.t

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.map_frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = p_local[0]
        odom.pose.pose.position.y = p_local[1]
        odom.pose.pose.position.z = p_local[2]
        # No orientation from GPS alone -- leave identity, and make sure
        # odom0_config in your EKF does NOT fuse orientation from this topic.
        odom.pose.pose.orientation.w = 1.0

        odom.pose.covariance = self._build_covariance(msg)

        self.pub.publish(odom)

    def _build_covariance(self, msg: NavSatFix):
        """Rotate NavSatFix's ENU position covariance into the local frame."""
        cov = np.array(msg.position_covariance, dtype=np.float64).reshape(3, 3)

        # Detect placeholder/zero covariance some drivers publish and fall back
        # to a conservative fixed estimate rather than telling the EKF GPS is
        # perfect (all-zero covariance) or garbage (COVARIANCE_TYPE_UNKNOWN).
        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN or np.allclose(cov, 0.0):
            var = self.default_position_stddev ** 2
            cov = np.diag([var, var, var * 4.0])  # vertical usually worse than horizontal

        # position_covariance is in ENU (east, north, up) local tangent order,
        # matching x=easting, y=northing before rotation. Rotate the xy block
        # by R (the same rotation applied to positions); z stays as-is since
        # T_world_utm's rotation block has no tilt (pure yaw about Z).
        R2 = self.R[:2, :2]
        cov_xy = cov[:2, :2]
        cov_xy_rot = R2 @ cov_xy @ R2.T

        pose_cov = np.full((6, 6), 1e6)  # huge covariance = "don't trust" for unfused fields
        pose_cov[0:2, 0:2] = cov_xy_rot
        pose_cov[2, 2] = cov[2, 2]

        return pose_cov.flatten().tolist()


def main(args=None):
    rclpy.init(args=args)
    node = GpsOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()