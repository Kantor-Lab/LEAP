# Pure math/library functions extracted from row_fitting.py.
# No rclpy.Node, no timer, no publisher — safe to import from any node
# (e.g. row_mission_node.py) without side effects.

from dataclasses import dataclass

import numpy as np


@dataclass
class Line:
    d: np.ndarray
    p: np.ndarray
    inlier_idx: np.ndarray = None
    error: float = float("inf")
    num_inliers: int = 0


def find_rows_ransac(tree_pts: np.ndarray) -> list[Line]:
    # tree_pts: Nx2 array
    max_lines = 100
    max_iters = 10000
    dist_thresh = 0.4
    min_inliers = 5

    vor_vert = tree_pts
    row_lines = []

    print("*** Starting RANSAC")

    for _ in range(max_lines):
        best_line = Line

        for _ in range(max_iters):
            if vor_vert.shape[0] < min_inliers:
                print("Not enough points remaining to form row")
                break

            rand_idx = np.random.choice(vor_vert.shape[0], 2, replace=False)
            pts = vor_vert[rand_idx, :]
            p1, p2 = pts
            v = p2 - p1

            diff = vor_vert - p1
            perp_dist = np.abs(diff[:, 0] * v[1] - diff[:, 1] * v[0]) / np.linalg.norm(v)

            inliers_idx = perp_dist < dist_thresh
            inliers = vor_vert[inliers_idx]

            if len(inliers) > min_inliers:
                centroid = np.mean(inliers, axis=0)
                U, S, Vt = np.linalg.svd(inliers - centroid)
                direction = Vt[0]
                err = S[1] ** 2 / inliers.shape[0]

                if len(inliers) > best_line.num_inliers or (
                    len(inliers) == best_line.num_inliers and err < best_line.error
                ):
                    best_line = Line(direction, centroid, inliers_idx, err, len(inliers))

        if best_line.num_inliers == 0:
            print(f"Row Searching stopped.  Found {len(row_lines)} rows")
            break

        row_lines.append(best_line)
        vor_vert = np.delete(vor_vert, best_line.inlier_idx, axis=0)

    return row_lines


def align_lines(line_list: list[Line]) -> list[Line]:
    ref_dir = line_list[0].d
    for line in line_list:
        if np.dot(line.d, ref_dir) < 0:
            line.d = -line.d
    return line_list


def get_avg_direction(line_list: list[Line]) -> np.ndarray:
    dirs = [line.d for line in line_list]
    avg_dir = np.mean(dirs, axis=0)
    avg_dir /= np.linalg.norm(avg_dir)
    return avg_dir


def clip_to_rotated_box(
    line_list: list[Line], tree_pts: np.ndarray, offset: float
) -> list[list[tuple]]:
    avg_dir = get_avg_direction(line_list)
    angle = np.arctan2(avg_dir[1], avg_dir[0])

    R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])

    plot_center = np.mean(tree_pts, axis=0)

    lines_local = [Line(R.T @ line.d, R.T @ (line.p - plot_center)) for line in line_list]
    tree_pts_local = (R.T @ (tree_pts - plot_center).T).T

    x_bounds = [np.min(tree_pts_local[:, 0]) - offset, np.max(tree_pts_local[:, 0]) + offset]
    y_bounds = [np.min(tree_pts_local[:, 1]) - offset, np.max(tree_pts_local[:, 1]) + offset]

    line_segs = clip_line_to_box(lines_local, x_bounds, y_bounds)

    world_segs = []
    for seg in line_segs:
        start_local, end_local = np.array(seg[0]), np.array(seg[1])
        start_world = (R @ start_local) + plot_center
        end_world = (R @ end_local) + plot_center
        world_segs.append([tuple(start_world), tuple(end_world)])

    return world_segs


def clip_line_to_box(line_list: list[Line], x_bounds: list, y_bounds: list) -> list[list[tuple]]:
    bounds = np.vstack([x_bounds, y_bounds])
    line_segs = []

    for line in line_list:
        t_min = float("-inf")
        t_max = float("inf")
        skip = False

        for i in range(2):
            p = line.p[i]
            d = line.d[i]
            box_min = bounds[i, 0]
            box_max = bounds[i, 1]

            if d == 0:
                if p < box_min or p > box_max:
                    print("line is parallel and outside bounding box")
                    skip = True
                    break
            else:
                t1 = (box_min - p) / d
                t2 = (box_max - p) / d
                t_min = max(t_min, min(t1, t2))
                t_max = min(t_max, max(t1, t2))

        if skip:
            continue
        if t_min > t_max:
            print("line misses the box")
            continue

        px, py = line.p
        dx, dy = line.d
        start_pt = (px + t_min * dx, py + t_min * dy)
        end_pt = (px + t_max * dx, py + t_max * dy)
        line_segs.append([start_pt, end_pt])

    return line_segs


def find_intersection(line1: Line, line2: Line) -> np.ndarray:
    A = np.array([line1.d, -line2.d]).T
    b = line2.p - line1.p

    if abs(np.linalg.det(A)) < 1e-9:
        return None
    t, u = np.linalg.solve(A, b)
    return line1.p + t * line1.d


def get_center_lines(all_lines: list[Line]) -> list[Line]:
    if len(all_lines) < 2:
        print("Not enough tree lines to find a center line")
        return []

    avg_dir = get_avg_direction(all_lines)
    perp_dir = np.array([-avg_dir[1], avg_dir[0]])
    sorted_lines = sorted(all_lines, key=lambda line: np.dot(line.p, perp_dir))

    center_lines = []
    for i in range(len(sorted_lines) - 1):
        line1 = sorted_lines[i]
        line2 = sorted_lines[i + 1]
        v = find_intersection(line1, line2)

        if v is not None:
            bisect_dir = line1.d + line2.d
            bisect_dir /= np.linalg.norm(bisect_dir)
            center_lines.append(Line(bisect_dir, v))
        else:
            midpoint = (line1.p + line2.p) / 2.0
            center_lines.append(Line(line1.d, midpoint))

    return center_lines


def get_waypoints(row_lines: list, start_point):
    waypoints = []
    headings = []
    curr_pt = np.array(start_point)

    row_lines = list(row_lines)  # local copy, since we mutate it

    for _ in range(len(row_lines)):
        tdistances = np.array(
            [
                [
                    np.linalg.norm(curr_pt - np.array(line[0])),
                    np.linalg.norm(curr_pt - np.array(line[1])),
                ]
                for line in row_lines
            ]
        )

        next_line, line_end = np.unravel_index(tdistances.argmin(), tdistances.shape)

        waypoints += [
            np.array(row_lines[next_line][line_end]),
            np.array(row_lines[next_line][np.abs(line_end - 1)]),
        ]
        headings += 2 * [
            (waypoints[-1] - waypoints[-2]) / np.linalg.norm(waypoints[-1] - waypoints[-2])
        ]
        curr_pt = waypoints[-1]

        del row_lines[next_line]

    return np.array(waypoints), np.array(headings)


def two_pt_spline(locs, heads, f, N):
    """Cubic Hermite spline segment between 2 points given their headings."""
    M = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [-3, 3, -2, -1], [2, -2, 1, 1]])

    C_x = M @ np.hstack((locs[:, 0], f * heads[:, 0]))
    C_y = M @ np.hstack((locs[:, 1], f * heads[:, 1]))

    t = np.linspace(0, 1, N)
    T = np.array([np.ones(N), t, t**2, t**3])
    X = T.T @ C_x
    Y = T.T @ C_y

    return np.column_stack((X, Y))
