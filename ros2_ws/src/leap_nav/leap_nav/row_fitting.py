# given set of 2d coordinates, determine rows and output waypoints at each end for use in the spline planner

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
import math

# ---------------------------------------------------------
# Dataclasses and Math Functions (From row_fitting.py)
# ---------------------------------------------------------
from dataclasses import dataclass

@dataclass
class Line:
    d: np.ndarray
    p: np.ndarray
    inlier_idx: np.ndarray = None
    error: float = float('inf')
    num_inliers: int = 0

def find_rows_ransac(tree_pts: np.ndarray) -> list[Line]: 
    
    #tree_pts: Nx2 array
    
    ## Parameters used in algorithm
    # offset = 1 #m.  Set amount of extra x and y offset from the edge of the tree points
    max_lines = 100 # maximum number of rows to search for.  Set high so it will likely not be reached
    max_iters = 10000 # number of RANSAC iterations to attempt
    dist_thresh = .4 #.5 #2 #m distance from RANSAC line to be considered an inlier
    min_inliers = 5 # min number of inliers to RANSAC line for it to be considered valid

    vor_vert = tree_pts
    
    row_lines = []

    #RANSAC start
    print('*** Starting RANSAC')

    for i in range(max_lines):

        best_line = Line
        # best_err = float('inf')

        for j in range(max_iters):

            if vor_vert.shape[0] < min_inliers:
                print('Not enough points remaining to form row')
                break

            # select 2 unique random indices from the voronoi vertices. (2 is min points to compute line)
            rand_idx = np.random.choice(vor_vert.shape[0], 2, replace=False)  
            pts = vor_vert[rand_idx, :] # get corresponding voronoi vertex points
            
            # print(f'1)  x: {pts[0,0]} | y: {pts[0,1]}')
            # print(f'2)  x: {pts[1,0]} | y: {pts[1,1]}')
            p1, p2 = pts
            v = p2-p1 # direction vector

            diff = vor_vert - p1
            perp_dist = np.abs(diff[:,0]*v[1] - diff[:,1]*v[0]) / np.linalg.norm(v) # perpendicular distance from all points to candidate line

            inliers_idx = perp_dist < dist_thresh
            inliers = vor_vert[inliers_idx]

            if len(inliers) > min_inliers:
                # compute the best fit line to the inlier data

                centroid = np.mean(inliers, axis=0)
                U, S, Vt = np.linalg.svd(inliers - centroid)
                direction = Vt[0]

                err = S[1]**2 / inliers.shape[0] # mean squared residual

                if len(inliers) > best_line.num_inliers or len(inliers) == best_line.num_inliers and err < best_line.error:
                    # best_err = err
                    best_line = Line(direction, centroid, inliers_idx, err, len(inliers))

        if best_line.num_inliers == 0 :
            print(f'Row Searching stopped.  Found {len(row_lines)} rows')
            break
        
        row_lines.append(best_line)
        # remove inliers from data and repeat
        vor_vert = np.delete(vor_vert, best_line.inlier_idx, axis=0) #removes the inliers from the search

    return row_lines

def align_lines(line_list: list[Line]) -> list[Line]:
    
    # ensure that all line vectors are pointing in the same direction
    ref_dir = line_list[0].d
    aligned_dirs = []
    for line in line_list:
        if np.dot(line.d, ref_dir) < 0:
            line.d = -line.d # modifies line to be aligned if not
            aligned_dirs.append(-line.d)
        else:
            aligned_dirs.append(line.d)
    
    # find the average direction vector of the aligned lines
    # avg_dir = np.mean(aligned_dirs, axis=0)
    # avg_dir /= np.linalg.norm(avg_dir)

    return line_list

def get_avg_direction(line_list: list[Line]) -> np.ndarray:
    # lines must be aligned to use this.

    dirs = [line.d for line in line_list]

    avg_dir = np.mean(dirs, axis=0)
    avg_dir /= np.linalg.norm(avg_dir)

    return avg_dir #2-vector

def clip_to_rotated_box(line_list: list[Line], 
                        tree_pts: np.ndarray, 
                        offset: int) -> list[list[tuple[float,float]]]:
    

    avg_dir = get_avg_direction(line_list)

    angle = np.arctan2(avg_dir[1], avg_dir[0])

    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle), np.cos(angle)]])
    
    plot_center = np.mean(tree_pts,axis=0)

    lines_local = [Line(R.T @ line.d, R.T @ (line.p - plot_center)) for line in line_list]

    tree_pts_local = (R.T @ (tree_pts - plot_center).T).T

    x_bounds = [np.min(tree_pts_local[:,0])-offset, np.max(tree_pts_local[:,0])+offset]
    y_bounds = [np.min(tree_pts_local[:,1])-offset, np.max(tree_pts_local[:,1])+offset]

    line_segs = clip_line_to_box(lines_local, x_bounds, y_bounds)

    world_segs = []
    for seg in line_segs:
        start_local, end_local = np.array(seg[0]), np.array(seg[1])

        start_world = (R @ start_local) + plot_center
        end_world = (R @ end_local) + plot_center

        world_segs.append([tuple(start_world), tuple(end_world)])


    return world_segs




def clip_line_to_box(line_list: list[Line],
                     x_bounds: list,
                     y_bounds: list) -> list[list[tuple[float,float]]]:
    # based on Liang-Barsky algorithm

    bounds = np.vstack([x_bounds,y_bounds])
    # print(bounds)

    line_segs = []

    for line in line_list:

        t_min = float('-inf')
        t_max = float('inf')

        for i in range(2):  # loop through x and y
            p = line.p[i]
            d = line.d[i]
            box_min = bounds[i, 0]
            box_max = bounds[i, 1]

            if d == 0:
                # line is parallel
                if p < box_min or p > box_max:
                    print('line is parallel and outside bounding box')
                    break
            else:
                t1 = (box_min - p) / d
                t2 = (box_max - p) / d

                t_entry = min(t1,t2)
                t_exit = max(t1,t2)

                t_min = max(t_min, t_entry)
                t_max = min(t_max, t_exit)
        
        if t_min > t_max:
            print('line misses the box')
            continue
        
        px, py = line.p
        dx,dy = line.d
        start_pt = (px + t_min * dx, py + t_min * dy)
        end_pt = (px + t_max * dx, py + t_max * dy)

        #list of list of tuples
        line_segs.append([start_pt, end_pt])

    return line_segs


def find_intersection(line1: Line, line2: Line) -> np.ndarray:

    # p1 + t*d1 = p2 + u*d2
    # [d1, -d2]*[t,u].T = (p2-p1)
    # Ax = b
    A = np.array([line1.d, -line2.d]).T
    b = line2.p - line1.p

    if abs(np.linalg.det(A)) < 1e-9:
        return None
    t,u = np.linalg.solve(A,b)

    return line1.p + t*line1.d

def get_center_lines(all_lines: list[Line]) -> list[Line]:
    if len(all_lines)<2:
        print('Not enough tree lines to find a center line')
        return []

    avg_dir = get_avg_direction(all_lines)

    perp_dir = np.array([-avg_dir[1], avg_dir[0]]) # vector perpendicular to average direction
    # projects each line onto the perp_vector and sorts to get them in a sorted order
    sorted_lines = sorted(all_lines, key=lambda line: np.dot(line.p, perp_dir)) 

    center_lines = []
    for i in range(len(sorted_lines)-1):
        line1 = sorted_lines[i]
        line2 = sorted_lines[i+1]

        v = find_intersection(line1, line2)

        if v is not None: 
            # lines intersect somewhere, use angle bisection
            bisect_dir = line1.d + line2.d
            bisect_dir /= np.linalg.norm(bisect_dir)
            center_lines.append(Line(bisect_dir,v))

        else:
            # lines are parallel, so can use either direction
            midpoint = (line1.p + line2.p) /2.0
            center_lines.append(Line(line1.d, midpoint))

    return center_lines


def get_waypoints(row_lines: list[list[tuple[float,float]]], 
                  start_point):

    waypoints = []
    headings = []
    curr_pt = np.array(start_point)

    for i in range(len(row_lines)):
        # this computes the distance from the current point to each end point of each line.  Distances are returned as a np array
        tdistances = np.array([[np.linalg.norm(curr_pt - np.array(line[0])), 
                                np.linalg.norm(curr_pt - np.array(line[1]))] 
                                for line in row_lines])
        
        # find the row and column of the minimum value
        next_line, line_end = np.unravel_index(tdistances.argmin(), tdistances.shape)

        
        #add the found nearest endpoint and the other end of the same line
        waypoints += [np.array(row_lines[next_line][line_end]), 
                      np.array(row_lines[next_line][np.abs(line_end-1)])]

        #add the headings at each waypoint (same value twice)
        headings += 2* [(waypoints[-1] - waypoints[-2]) / np.linalg.norm(waypoints[-1] -waypoints[-2])]
        # set the current point to the last waypoint
        # curr_pt = np.array(waypoints[-1])
        curr_pt = waypoints[-1]
        
        #remove the found line
        del row_lines[next_line]

    return np.array(waypoints), np.array(headings)


def two_pt_spline(locs,heads,f, N):
    """ Computes a cubic Hermite spline segment between 2 points given their headings

    Args:
        locs: numpy array of 2 point locations: [x,y]
        heads: numpy array of 2 point headings: [dx, dy]
        f: scale factor multiplied to heading directions
        N: num of interpolation points of spline

    Returns:
        spline
    """
    M = np.array([[1, 0, 0, 0],
                  [0, 0, 1, 0],
                  [-3, 3, -2, -1],
                  [2, -2, 1, 1]])

    # print(f'Px: {np.hstack((locs[:,0],f*heads[:,0]))}')
    # print(f'Py: {np.hstack((locs[:,1],f*heads[:,1]))}')

    C_x = M @ np.hstack((locs[:,0],f*heads[:,0]))
    C_y = M @ np.hstack((locs[:,1],f*heads[:,1]))
    
    
    # print(f'Cx: {C_x}')
    # print(f'Cy: {C_y}')

    t = np.linspace(0,1,N)
    T = np.array([np.ones(N), t, t**2, t**3])
    X = T.T @ C_x
    Y = T.T @ C_y

    spline = np.column_stack((X,Y))
    return spline

# ---------------------------------------------------------
# ROS 2 Node Definition
# ---------------------------------------------------------

class RowPlannerNode(Node):
    def __init__(self):
        super().__init__('leap_row_planner')
        
        # ROS 2 Parameters
        self.declare_parameter('offset', 2.0)
        self.declare_parameter('min_trees_in_plot', 10)
        self.declare_parameter('frame_id', 'map')
        
        self.offset = self.get_parameter('offset').value
        self.min_trees = self.get_parameter('min_trees_in_plot').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Publisher for Nav2 to consume the generated path
        self.path_pub = self.create_publisher(Path, '/planned_row_path', 10)
        
        # Example Timer to simulate receiving data and generating a path
        # In a complete stack, this would likely be triggered by a Service callback
        # or a Subscription to a clustered PointCloud2 / PoseArray of tree centroids.
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info("Row Planner Node initialized. Waiting for tree data...")

    def generate_nav2_path(self, spline_points: np.ndarray) -> Path:
        """Converts the numpy spline array into a nav_msgs/Path."""
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id

        for i in range(spline_points.shape[0]):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(spline_points[i, 0])
            pose.pose.position.y = float(spline_points[i, 1])
            pose.pose.position.z = 0.0
            
            # Calculate heading (yaw) for the quaternion based on the next point
            if i < spline_points.shape[0] - 1:
                dx = spline_points[i+1, 0] - spline_points[i, 0]
                dy = spline_points[i+1, 1] - spline_points[i, 1]
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0 # Maintain last heading for the final point
                
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            path_msg.poses.append(pose)
            
        return path_msg

    def timer_callback(self):
        # -------------------------------------------------------------
        # Mock Data Injection: Replace this block with your actual 
        # tree centroid subscriptions or service requests.
        # -------------------------------------------------------------
        # Example dummy data simulating `tree_pts0` from your pickle file
        tree_pts = np.array([
            [0, 0], [0, 5], [0, 10], 
            [5, 0], [5, 5], [5, 10], 
            [10, 0], [10, 5], [10, 10]
        ])
        start_point = np.array([-5, -5])
        start_heading = np.array([np.cos(45*np.pi/180), np.sin(45*np.pi/180)])
        # -------------------------------------------------------------

        if tree_pts.shape[0] < self.min_trees:
            self.get_logger().warn('Not enough trees for row detection')
            return

        # 1. Extract Rows
        row_lines = find_rows_ransac(tree_pts)
        if not row_lines:
            self.get_logger().warn('No rows found.')
            return

        # 2. Process Lines
        row_lines = align_lines(row_lines)
        tree_line_segs = clip_to_rotated_box(row_lines, tree_pts, self.offset)
        
        # 3. Get Waypoints & Generate Spline
        waypoints, headings = get_waypoints(tree_line_segs, start_point)
        waypoints = np.vstack((start_point, waypoints))
        headings = np.vstack((start_heading, headings))

        full_spline_list = []
        for i in range(waypoints.shape[0]-1):
            spline_seg = two_pt_spline(waypoints[i:i+2, :], headings[i:i+2, :], 8, 100)
            full_spline_list.append(spline_seg)

        full_spline_np = np.vstack(full_spline_list)

        # 4. Convert to ROS 2 Path and Publish
        path_msg = self.generate_nav2_path(full_spline_np)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published agricultural path with {len(path_msg.poses)} poses.")
        
        # Stop timer after one successful run if this is meant to be a one-shot generation
        self.timer.cancel() 

def main(args=None):
    rclpy.init(args=args)
    node = RowPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()