#!/usr/bin/env python3
import numpy as np
from dataclasses import dataclass
from typing import Optional

@dataclass
class OrchardRow:
    """
    A detected orchard row represented as a line in map frame.

    :param point: a point on the line (x, y) in map frame (metres)
    :param direction: unit vector (dx, dy) along the row direction
    :param inliers: array of (x, y) obstacle points supporting this line
    :param residual: mean perpendicular distance of inliers to the line (metres)
    """
    point: np.ndarray        # shape (2,)
    direction: np.ndarray    # shape (2,), unit vector
    inliers: np.ndarray      # shape (N, 2)
    residual: float

def get_orchard_row_from_ransac(points: np.ndarray, inlier_threshold: float, min_inliers: int, max_iterations: int) -> Optional[OrchardRow]:
    """
    Fit a line to a 2D point cloud using RANSAC.

    :param points: (N, 2) array of (x, y) positions
    :param inlier_threshold: max perpendicular distance (metres) to count as inlier
    :param min_inliers: minimum inliers required to accept a line hypothesis
    :param max_iterations: number of RANSAC iterations
    :returns: OrchardRow if a line was found, else None
    """
    if len(points) < 2:
        return None

    best_inlier_mask = None
    best_inlier_count = 0
    rng = np.random.default_rng(seed=42)

    for _ in range(max_iterations):
        # Sample 2 random points to define a line hypothesis
        idx = rng.choice(len(points), size=2, replace=False)
        p1, p2 = points[idx[0]], points[idx[1]]

        direction = p2 - p1
        length = np.linalg.norm(direction)
        if length < 1e-6:
            continue
        direction = direction / length

        # Perpendicular distance from all points to this line
        diff = points - p1
        perp_dist = np.abs(diff[:, 0] * (-direction[1]) + diff[:, 1] * direction[0])

        inlier_mask = perp_dist < inlier_threshold
        inlier_count = np.sum(inlier_mask)

        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            best_inlier_mask = inlier_mask

    if best_inlier_count < min_inliers:
        return None

    # Refit line to all inliers using PCA for a least-squares direction
    inliers = points[best_inlier_mask]
    centroid = inliers.mean(axis=0)
    _, _, Vt = np.linalg.svd(inliers - centroid)
    direction = Vt[0]  # principal axis = row direction

    # Recompute residual on final fit
    diff = inliers - centroid
    perp_dist = np.abs(diff[:, 0] * (-direction[1]) + diff[:, 1] * direction[0])
    residual = perp_dist.mean()

    return OrchardRow(point=centroid, direction=direction, inliers=inliers, residual=residual)


def detect_orchard_rows(
    obstacle_positions: list[tuple[float, float]],
    inlier_threshold: float = 0.2,
    min_inliers: int = 10,
    max_iterations: int = 100,
    min_row_separation: float = 1.5,
) -> list[OrchardRow]:
    """
    Fit up to two orchard row lines from obstacle positions using sequential RANSAC.
    Returns 0, 1, or 2 OrchardRow objects — never raises on partial detection.

    :param obstacle_positions: list of (x, y) map frame positions from get_obstacle_positions()
    :param inlier_threshold: max perpendicular distance (metres) to count as inlier
    :param min_inliers: minimum obstacle points required to accept a line
    :param max_iterations: RANSAC iterations per line fit
    :param min_row_separation: minimum perpendicular distance (metres) between the two
        fitted lines. Points too close to the first line are excluded before fitting
        the second, preventing two lines from fitting to the same row.
    :returns: list of OrchardRow, length 0, 1, or 2
    """
    if not obstacle_positions:
        return []

    points = np.array(obstacle_positions)
    rows = []

    for i in range(2):
        row = get_orchard_row_from_ransac(points, inlier_threshold, min_inliers, max_iterations)
        if row is None:
            break
        rows.append(row)

        if i == 0:
            # Remove all points within min_row_separation of the first line,
            # not just its inliers — this prevents the second fit drifting onto
            # the same row via outlier points that RANSAC didn't claim
            diff = points - row.point
            perp_dist = np.abs(
                diff[:, 0] * (-row.direction[1]) + diff[:, 1] * row.direction[0]
            )
            points = points[perp_dist >= min_row_separation]

    return rows