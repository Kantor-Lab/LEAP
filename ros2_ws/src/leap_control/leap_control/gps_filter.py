#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry


class GpsFilterNode(Node):
    """Cross-checks incoming GPS against the local (IMU+wheel) EKF before
    letting it reach the global filter.

    Design notes (see discussion history):
    - The accept/reject tolerance is sized from GPS epoch noise plus a bound
      on local-EKF drift growth since the anchor, NOT from the global
      filter's covariance. Global covariance answers "how far should a
      *self-consistent* reading be allowed to move the state" (the EKF's own
      innovation gating already handles that) -- it says nothing about
      whether a reading is self-consistent in the first place, and scaling
      this check with it would make bad GPS *easier* to accept exactly when
      the system is already confused.
    - The tolerance grows with elapsed time since the anchor (bounded local
      drift really does grow with time) but is capped. Past the cap, the
      local reference is no longer trustworthy enough to judge anything
      against, so the anchor is invalidated and a fresh streak is required
      before GPS is trusted again -- growing the tolerance further would
      just make the check meaningless instead of lenient.
    - The anchor is never updated on a single lone reading. Promotion to
      anchor requires a short streak of mutually-consistent consecutive
      readings, so a single bad epoch arriving right as ICP starts to fail
      can't become the reference everything else is measured against.
    """

    def __init__(self):
        super().__init__('gps_filter')

        # --- ICP health monitoring (diagnostic / gating trigger only) ---
        self.declare_parameter('cov_threshold', 0.75)  # m^2, global filter pos variance

        # --- Self-consistency tolerance model ---
        # tolerance(dt) = min(max_tolerance,
        #                     consistency_sigma * (gps_noise_std + odom_drift_rate * dt))
        self.declare_parameter('gps_noise_std', 0.05)       # m, epoch-to-epoch RTK jitter budget
        self.declare_parameter('odom_drift_rate', 0.03)     # m/s, bound on local EKF drift growth
        self.declare_parameter('consistency_sigma', 4.0)    # multiplier on the noise budget
        self.declare_parameter('max_tolerance', 1.0)        # m, hard cap -- see class docstring

        # --- Anchor lifecycle ---
        self.declare_parameter('baseline_streak_required', 3)   # consecutive consistent reads to (re)confirm anchor
        self.declare_parameter('reanchor_period_sec', 10.0)     # how often to refresh the anchor while trusted
        self.declare_parameter('anchor_max_age_sec', 30.0)      # beyond this, anchor is stale -> force re-validation

        self.cov_threshold = self.get_parameter('cov_threshold').value
        self.gps_noise_std = self.get_parameter('gps_noise_std').value
        self.odom_drift_rate = self.get_parameter('odom_drift_rate').value
        self.consistency_sigma = self.get_parameter('consistency_sigma').value
        self.max_tolerance = self.get_parameter('max_tolerance').value
        self.baseline_streak_required = self.get_parameter('baseline_streak_required').value
        self.reanchor_period_sec = self.get_parameter('reanchor_period_sec').value
        self.anchor_max_age_sec = self.get_parameter('anchor_max_age_sec').value

        # State
        self.latest_local_odom = None
        self.global_cov_high = False

        # Confirmed anchor: the last validated (gps_pose, odom_pose, stamp_sec)
        # everything is measured against. None until a streak confirms one.
        self.anchor_gps_pose = None
        self.anchor_odom_pose = None
        self.anchor_stamp_sec = None

        # Rolling streak-candidate state (runs continuously, independent of
        # whether ICP is currently healthy, so a validated anchor is always
        # ready by the time it's actually needed).
        self._prev_gps_pose = None
        self._prev_odom_pose = None
        self._prev_stamp_sec = None
        self._streak_count = 0

        # Subscribers
        self.create_subscription(Odometry, '/odometry/global', self.global_odom_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.local_odom_cb, 10)
        self.create_subscription(Odometry, '/fix/transformed', self.gps_cb, 10)

        # Publisher
        self.gps_pub = self.create_publisher(Odometry, '/odometry/gps', 10)

        self.get_logger().info("GPS self-consistency filter node initialized.")

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def global_odom_cb(self, msg: Odometry):
        # Covariance is a 36-element array (6x6 matrix). Index 0 is X variance, 7 is Y variance.
        cov_x = msg.pose.covariance[0]
        cov_y = msg.pose.covariance[7]
        max_var = max(cov_x, cov_y)
        # Used only to decide *when* to start actively filtering GPS (i.e. as
        # a trigger), never to size the self-consistency tolerance itself.
        self.global_cov_high = max_var > self.cov_threshold

    def local_odom_cb(self, msg: Odometry):
        self.latest_local_odom = msg.pose.pose

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _distance(pose1, pose2):
        if pose1 is None or pose2 is None:
            return 0.0
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.hypot(dx, dy)

    @staticmethod
    def _stamp_sec(header_stamp):
        return Time.from_msg(header_stamp).nanoseconds / 1e9

    def _tolerance(self, dt_sec):
        dt_sec = max(0.0, dt_sec)
        budget = self.consistency_sigma * (self.gps_noise_std + self.odom_drift_rate * dt_sec)
        return min(self.max_tolerance, budget)

    def _is_consistent(self, gps_pose_a, odom_pose_a, stamp_a_sec,
                        gps_pose_b, odom_pose_b, stamp_b_sec):
        dt = abs(stamp_b_sec - stamp_a_sec)
        gps_disp = self._distance(gps_pose_a, gps_pose_b)
        odom_disp = self._distance(odom_pose_a, odom_pose_b)
        tolerance = self._tolerance(dt)
        return abs(gps_disp - odom_disp) <= tolerance, tolerance

    # ------------------------------------------------------------------
    # Anchor lifecycle (runs on every GPS message, regardless of ICP health)
    # ------------------------------------------------------------------
    def _update_streak_and_maybe_promote_anchor(self, gps_pose, odom_pose, stamp_sec):
        if self._prev_gps_pose is not None:
            consistent, _ = self._is_consistent(
                self._prev_gps_pose, self._prev_odom_pose, self._prev_stamp_sec,
                gps_pose, odom_pose, stamp_sec)
            self._streak_count = self._streak_count + 1 if consistent else 0
        else:
            self._streak_count = 0

        self._prev_gps_pose = gps_pose
        self._prev_odom_pose = odom_pose
        self._prev_stamp_sec = stamp_sec

        anchor_absent_or_stale = (
            self.anchor_stamp_sec is None
            or (stamp_sec - self.anchor_stamp_sec) >= self.reanchor_period_sec
        )

        if self._streak_count >= self.baseline_streak_required and anchor_absent_or_stale:
            self.anchor_gps_pose = gps_pose
            self.anchor_odom_pose = odom_pose
            self.anchor_stamp_sec = stamp_sec
            self.get_logger().info(
                "GPS anchor (re)confirmed after {0}-read consistent streak.".format(self._streak_count))

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------
    def gps_cb(self, msg: Odometry):
        if self.latest_local_odom is None:
            self.get_logger().warn("Waiting for /odometry/filtered...", throttle_duration_sec=2.0)
            return

        gps_pose = msg.pose.pose
        odom_pose = self.latest_local_odom
        stamp_sec = self._stamp_sec(msg.header.stamp)

        # Always maintain/validate the anchor in the background, independent
        # of ICP health, so a trustworthy reference already exists by the
        # time ICP actually fails -- rather than grabbing whatever the last
        # raw sample happened to be at the moment of failure.
        self._update_streak_and_maybe_promote_anchor(gps_pose, odom_pose, stamp_sec)

        if not self.global_cov_high:
            # ICP is healthy; the global filter's own innovation gating is
            # the relevant safeguard here. Pass GPS through.
            self.gps_pub.publish(msg)
            self.get_logger().info("ICP healthy. GPS passed through.", throttle_duration_sec=5.0)
            return

        # ICP is unhealthy -- this is where the self-consistency filter
        # actually needs to do its job.
        if self.anchor_gps_pose is None:
            self.get_logger().warn(
                "ICP unhealthy and no validated GPS anchor yet -- rejecting GPS until a "
                "consistent streak is established.",
                throttle_duration_sec=2.0)
            return

        anchor_age = stamp_sec - self.anchor_stamp_sec
        if anchor_age >= self.anchor_max_age_sec:
            # The local reference is too old to mean anything; growing the
            # tolerance further would just make the check meaningless.
            # Invalidate and force a fresh streak before trusting GPS again.
            self.get_logger().warn(
                "GPS anchor is stale ({0:.1f}s old) -- invalidating and requiring "
                "re-validation before trusting GPS again.".format(anchor_age),
                throttle_duration_sec=2.0)
            self.anchor_gps_pose = None
            self.anchor_odom_pose = None
            self.anchor_stamp_sec = None
            return

        consistent, tolerance = self._is_consistent(
            self.anchor_gps_pose, self.anchor_odom_pose, self.anchor_stamp_sec,
            gps_pose, odom_pose, stamp_sec)

        if consistent:
            self.gps_pub.publish(msg)
        else:
            gps_disp = self._distance(self.anchor_gps_pose, gps_pose)
            odom_disp = self._distance(self.anchor_odom_pose, odom_pose)
            self.get_logger().warn(
                "GPS rejected! |gps_disp - odom_disp| = {0:.2f}m > tolerance {1:.2f}m "
                "(anchor age {2:.1f}s)".format(abs(gps_disp - odom_disp), tolerance, anchor_age),
                throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = GpsFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()