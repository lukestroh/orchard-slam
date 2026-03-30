#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import PoseStamped, Pose
from builtin_interfaces.msg import Time


def _resolve_direction(direction: np.ndarray, robot_heading: float) -> np.ndarray:
    """
    Resolve the sign ambiguity of a RANSAC direction vector by ensuring it
    points in the same general direction as the robot's current heading.
    """
    heading_vec = np.array([np.cos(robot_heading), np.sin(robot_heading)])
    if np.dot(direction, heading_vec) < 0:
        return -direction
    return direction


def _project_onto_line(point: np.ndarray, line_point: np.ndarray, line_dir: np.ndarray) -> np.ndarray:
    t = np.dot(point - line_point, line_dir)
    return line_point + t * line_dir


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    """
    Convert a yaw angle to a quaternion (x, y, z, w).

    :param yaw: rotation about z-axis in radians
    :returns: (x, y, z, w) quaternion tuple
    """
    return 0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)


def get_row_goal(
    rows: list,
    robot_pose:Pose,
    robot_heading: float,
    goal_distance: float = 4.0,
    expected_row_width: float = 4.0,
) -> tuple[np.ndarray, float]:
    robot_pos = np.array([robot_pose.position.x, robot_pose.position.y])
    """TODO: this needs to handle more edge cases
    - no rows
    - assign confidence score based on how much it has moved over the last frames, also
      bias towards closer to current robot orientation
    - if rows are uneven need to continue along longer one for full coverage
    
    """
    # Two rows
    if len(rows) == 2:
        row_a, row_b = rows
        dir_a = _resolve_direction(row_a.direction, robot_heading)
        dir_b = _resolve_direction(row_b.direction, robot_heading)
        centerline_dir = dir_a + dir_b
        centerline_dir /= np.linalg.norm(centerline_dir)

        proj_a = _project_onto_line(robot_pos, row_a.point, dir_a)
        proj_b = _project_onto_line(robot_pos, row_b.point, dir_b)
        centerline_origin = (proj_a + proj_b) / 2.0
    # One-row
    else:
        row = rows[0]
        centerline_dir = _resolve_direction(row.direction, robot_heading)

        proj = _project_onto_line(robot_pos, row.point, centerline_dir)

        # Perpendicular left of the resolved forward direction
        perp_left = np.array([-centerline_dir[1], centerline_dir[0]])

        # Offset toward whichever perpendicular side the robot is NOT on,
        # i.e. toward the missing row
        to_row = row.point - robot_pos
        if np.dot(to_row, perp_left) > 0:
            # Detected row is to the left, so missing row is to the right
            lateral_offset = -perp_left
        else:
            lateral_offset = perp_left

        centerline_origin = proj + lateral_offset * (expected_row_width / 2.0)

    # Place goal ahead of the robot's projection onto the centerline
    robot_proj_t = np.dot(robot_pos - centerline_origin, centerline_dir)
    goal_pos = centerline_origin + centerline_dir * (robot_proj_t + goal_distance)
    yaw = np.arctan2(centerline_dir[1], centerline_dir[0])

    return (goal_pos, yaw)