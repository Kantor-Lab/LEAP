#!/usr/bin/env python3
"""
leap_row_mission: waits for init, drives to the start of a tree-row path via
navigate_to_pose, then hands off directly to the controller_server's
follow_path action to run the precomputed spline (bypassing the planner/BT).

Requires row_fitting_lib.py (the pure math/library version of row_fitting.py,
with the RowPlannerNode / rclpy.init() / timer stuff stripped out) importable
on PYTHONPATH.
"""

import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node

from leap_nav.row_fitting_lib import (
    align_lines,
    clip_to_rotated_box,
    find_rows_ransac,
    get_waypoints,
    two_pt_spline,
    get_center_lines
)


class MissionState:
    WAIT = 0
    NAV_TO_ROW_START = 1
    FOLLOW_ROW_PATH = 2
    DONE = 3


class RowMissionNode(Node):
    def __init__(self):
        super().__init__("leap_row_mission")

        self.declare_parameter("init_wait_sec", 30.0)
        self.declare_parameter("offset", 2.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("controller_id", "FollowPath")

        self.init_wait_sec = self.get_parameter("init_wait_sec").value
        self.offset = self.get_parameter("offset").value
        self.frame_id = self.get_parameter("frame_id").value
        self.controller_id = self.get_parameter("controller_id").value

        self.state = MissionState.WAIT
        self.start_time = self.get_clock().now()
        self.row_path_msg = None
        self.row_start_pose = None

        self.debug_path_pub = self.create_publisher(Path, '/boustrephodon', 10)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")

        # Single lightweight timer just to gate the initial wait; everything
        # after that is driven by action-callback chaining, not polling.
        self.wait_timer = self.create_timer(0.5, self._wait_tick)

        self.get_logger().info(
            f"leap_row_mission up, waiting {self.init_wait_sec:.0f}s before starting mission"
        )

    # ------------------------------------------------------------------
    # Step 1: initial wait
    # ------------------------------------------------------------------
    def _wait_tick(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.init_wait_sec:
            return

        self.wait_timer.cancel()
        self.get_logger().info("Init wait complete, generating row path")
        self.generate_row_path()
        self.send_nav_to_pose_goal()

    # ------------------------------------------------------------------
    # Row path generation (mirrors row_fitting.py's timer_callback body)
    # ------------------------------------------------------------------
    def generate_row_path(self):
        tree_pts = np.array([
            [43, 63], [43, 65], [43, 67], [43, 69], [43, 71], [43, 73],
            [45, 63], [45, 65], [45, 67], [45, 69], [45, 71], [45, 73],
            [47, 63], [47, 65], [47, 67], [47, 69], [47, 71], [47, 73],
            [49, 63], [49, 65], [49, 67], [49, 69], [49, 71], [49, 73],
            [51, 63], [51, 65], [51, 67], [51, 69], [51, 71], [51, 73],
        ])
        start_point = np.array([44.0, 61.0])
        start_heading = np.array([0.0, 1.0])

        row_lines = find_rows_ransac(tree_pts)
        row_lines = align_lines(row_lines)

        center_lines = get_center_lines(row_lines)
        if not center_lines:
            self.get_logger().error("Not enough tree rows to compute center lines")
            return

        tree_line_segs = clip_to_rotated_box(center_lines, tree_pts, self.offset)
        waypoints, headings = get_waypoints(tree_line_segs, start_point)
        waypoints = np.vstack((start_point, waypoints))
        headings = np.vstack((start_heading, headings))

        splines = []
        for i in range(waypoints.shape[0] - 1):
            seg = two_pt_spline(waypoints[i : i + 2, :], headings[i : i + 2, :], 15, 100)
            splines.append(seg)
        full_spline = np.vstack(splines)

        self.row_path_msg = self._spline_to_path(full_spline)
        self.row_start_pose = self.row_path_msg.poses[0]
        self.debug_path_pub.publish(self.row_path_msg)
        self.get_logger().info(
            f"Row path generated: {len(self.row_path_msg.poses)} poses, "
            f"start=({self.row_start_pose.pose.position.x:.2f}, "
            f"{self.row_start_pose.pose.position.y:.2f})"
        )

    def _spline_to_path(self, spline_points: np.ndarray) -> Path:
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        yaw = 0.0
        for i in range(spline_points.shape[0]):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(spline_points[i, 0])
            pose.pose.position.y = float(spline_points[i, 1])
            pose.pose.position.z = 0.0

            if i < spline_points.shape[0] - 1:
                dx = spline_points[i + 1, 0] - spline_points[i, 0]
                dy = spline_points[i + 1, 1] - spline_points[i, 1]
                yaw = math.atan2(dy, dx)
            # else: keep last yaw for the final pose

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)

        return path

    # ------------------------------------------------------------------
    # Step 2: navigate_to_pose to the row start
    # ------------------------------------------------------------------
    def send_nav_to_pose_goal(self):
        self.state = MissionState.NAV_TO_ROW_START
        self.get_logger().info("Waiting for navigate_to_pose server...")
        self.nav_to_pose_client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = self.row_start_pose
        self.get_logger().info("Sending NavigateToPose goal (row start)")
        send_future = self.nav_to_pose_client.send_goal_async(goal)
        send_future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("NavigateToPose goal rejected, aborting mission")
            self._finish()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Reached row start, handing off to FollowPath")
            self.send_follow_path_goal()
        else:
            self.get_logger().error(f"NavigateToPose failed, status={status}")
            self._finish()

    # ------------------------------------------------------------------
    # Step 3: hand off directly to controller_server's follow_path action
    # ------------------------------------------------------------------
    def send_follow_path_goal(self):
        self.state = MissionState.FOLLOW_ROW_PATH
        self.follow_path_client.wait_for_server()

        goal = FollowPath.Goal()
        goal.path = self.row_path_msg
        goal.controller_id = self.controller_id
        send_future = self.follow_path_client.send_goal_async(goal)
        send_future.add_done_callback(self._follow_goal_response_cb)

    def _follow_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("FollowPath goal rejected, aborting mission")
            self._finish()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._follow_result_cb)

    def _follow_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Row path complete")
        else:
            self.get_logger().error(f"FollowPath failed, status={status}")
        self._finish()

    # ------------------------------------------------------------------
    # Step 4: end
    # ------------------------------------------------------------------
    def _finish(self):
        self.state = MissionState.DONE
        self.get_logger().info("Mission finished, shutting down node")
        # give logging a moment to flush before tearing down
        self.create_timer(0.5, lambda: rclpy.shutdown())


def main(args=None):
    rclpy.init(args=args)
    node = RowMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
