import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from orchard_msgs.action import StartOrchardNavigation
from orchard_msgs.msg import OrchardNavState as OrchardNavStateMsg
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from orchard_slam_bringup.logger_node import LoggerNode
from orchard_nav.nav_state import OrchardNavState
from orchard_nav import orchard_row as ochr

import numpy as np
from threading import Lock


class OrchardNav(LoggerNode):
    def __init__(self):
        super().__init__("orchard_nav")

        # locks
        self._lock_local_costmap = Lock()
        self._lock_global_costmap = Lock()

        # Callback groups
        self._cb_group_reentrant = ReentrantCallbackGroup()

        # QoS profiles
        self._qos_orchard_nav_status = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._qos_initial_start_pose = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._qos_goal_pose = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
        )

        # Subscriptions
        self._sub_initial_start_pose = self.create_subscription(
            msg_type=PoseStamped,
            topic="/orchard_nav/initial_start_pose",
            callback=self._sub_cb_initial_start_pose,
            callback_group=self._cb_group_reentrant,
            qos_profile=self._qos_initial_start_pose,
        )
        self._sub_nav_cmd_vel = self.create_subscription(
            msg_type=Twist,
            topic="/cmd_vel_nav",
            callback=self._sub_cb_nav_cmd_vel,
            callback_group=self._cb_group_reentrant,
            qos_profile=10,
        )
        self._sub_local_costmap = self.create_subscription(
            msg_type=OccupancyGrid,
            topic="/local_costmap/costmap",
            callback=self._sub_cb_local_costmap,
            callback_group=self._cb_group_reentrant,
            qos_profile=10,
        )
        self._sub_global_costmap = self.create_subscription(
            msg_type=OccupancyGrid,
            topic="/global_costmap/costmap",
            callback=self._sub_cb_global_costmap,
            callback_group=self._cb_group_reentrant,
            qos_profile=10,
        )
        self._sub_map = self.create_subscription(
            msg_type=OccupancyGrid,
            topic="/map",
            callback=self._sub_cb_map,
            callback_group=self._cb_group_reentrant,
            qos_profile=10,
        )
        self._sub_odom = self.create_subscription(
            msg_type=Odometry,
            topic="/odometry/filtered/local",
            callback=self._sub_cb_odom,
            callback_group=self._cb_group_reentrant,
            qos_profile=10,
        )
        self._sub_orchard_nav_active_state = self.create_subscription(
            msg_type=OrchardNavStateMsg,
            topic="/orchard_nav/active_state",
            callback=self._sub_cb_orchard_nav_active_state,
            callback_group=self._cb_group_reentrant,
            qos_profile=self._qos_orchard_nav_status,
        )

        # Publishers
        self._pub_orchard_nav_next_state = self.create_publisher(
            msg_type=OrchardNavStateMsg,
            topic="/orchard_nav/next_state",
            callback_group=self._cb_group_reentrant,
            qos_profile=self._qos_orchard_nav_status,
        )
        self._pub_diff_drive_cmd_vel = self.create_publisher(
            msg_type=TwistStamped,
            topic="/diff_drive_controller/cmd_vel",
            callback_group=self._cb_group_reentrant,
            qos_profile=10,
        )
        self._pub_goal_pose = self.create_publisher(
            msg_type=PoseStamped,
            topic="/goal_pose",
            callback_group=self._cb_group_reentrant,
            qos_profile=self._qos_goal_pose,
        )
        self._pub_detected_row_markers = self.create_publisher(
            msg_type=MarkerArray,
            topic="/orchard_nav/detected_row_markers",
            callback_group=self._cb_group_reentrant,
            qos_profile=10,
        )

        # Action servers
        self._action_svr_explore = ActionServer(
            node=self,
            action_type=StartOrchardNavigation,
            action_name="/orchard_nav/explore",
            cancel_callback=self._action_cancel_cb_explore,
            goal_callback=self._action_goal_cb_explore,
            execute_callback=self._action_exec_cb_explore,
            callback_group=self._cb_group_reentrant,
        )
        self._action_svr_traverse_row = ActionServer(
            # navigate to a pose by publishing
            node=self,
            action_type=StartOrchardNavigation,
            action_name="/orchard_nav/traverse_row",
            cancel_callback=self._action_cancel_cb_traverse_row,
            goal_callback=self._action_goal_cb_traverse_row,
            execute_callback=self._action_exec_cb_traverse_row,
            callback_group=self._cb_group_reentrant,
        )
        self._action_svr_turn_row = ActionServer(
            node=self,
            action_type=StartOrchardNavigation,
            action_name="/orchard_nav/turn_row",
            cancel_callback=self._action_cancel_cb_turn_row,
            goal_callback=self._action_goal_cb_turn_row,
            execute_callback=self._action_exec_cb_turn_row,
            callback_group=self._cb_group_reentrant,
        )
        self._action_svr_return_home = ActionServer(
            node=self,
            action_type=StartOrchardNavigation,
            action_name="/orchard_nav/return_home",
            cancel_callback=self._action_cancel_cb_return_home,
            goal_callback=self._action_goal_cb_return_home,
            execute_callback=self._action_exec_cb_return_home,
            callback_group=self._cb_group_reentrant,
        )

        # Messages
        self._msg_laser_scan = None
        self._msg_local_costmap = None
        self._msg_global_costmap = None
        self._msg_occupancy_grid = None
        self._msg_robot_pose = None
        self._msg_orchard_nav_active_state = None
        self._msg_initial_start_pose = None

        # State variables
        self.nav_state = OrchardNavState.IDLE
        self.goal_pose = None
        self.robot_facing = None
        self.row_detected = False
        return

    def _sub_cb_odom(self, msg: Odometry):
        """Callback for robot's current position (if needed)"""
        self._msg_robot_pose = msg
        robot_pose = self._msg_robot_pose.pose.pose
        self.robot_pose = robot_pose
        self.robot_facing = np.arctan2(self.robot_pose.orientation.z, self.robot_pose.orientation.w) * 2.0

        return

    def _sub_cb_nav_cmd_vel(self, msg: Twist):
        """Callback for cmd_vel to track robot's current velocity (if needed)"""
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = Twist()
        twist_stamped.twist.linear.x = msg.linear.x
        twist_stamped.twist.angular.z = msg.angular.z
        self._pub_diff_drive_cmd_vel.publish(twist_stamped)  # Forward the cmd_vel message to the robot
        # self.get_logger().info(f"Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}")
        return

    def _sub_cb_laser_scan(self, msg: LaserScan):
        """Callback for laser scan data (obstacle detection)"""
        self._msg_laser_scan = msg
        return

    def _sub_cb_local_costmap(self, msg: OccupancyGrid):
        """Callback for local costmap data"""
        with self._lock_local_costmap:
            self._msg_local_costmap = msg
        return
    
    def _sub_cb_map(self, msg: OccupancyGrid):
        """Callback for map data"""
        self._msg_occupancy_grid = msg
        return
    
    def _sub_cb_global_costmap(self, msg: OccupancyGrid):
        """Callback for global costmap data"""
        with self._lock_global_costmap:
            self._msg_global_costmap = msg
        return

    def _sub_cb_orchard_nav_active_state(self, msg: OrchardNavStateMsg):
        """Callback for orchard navigation active state updates"""
        self._msg_orchard_nav_active_state = msg
        return
    
    def _sub_cb_initial_start_pose(self, msg: PoseStamped):
        """Callback for receiving the robot's initial pose at the start of navigation"""
        self._msg_initial_start_pose = msg
        return
    
    def _action_goal_cb_explore(self, goal_handle):
        self.get_logger().info("Received explore goal request")
        return GoalResponse.ACCEPT
    
    def _action_cancel_cb_explore(self, goal_handle):
        self.get_logger().info("Received explore goal cancel request")
        return CancelResponse.ACCEPT
    
    def _action_exec_cb_explore(self, goal_handle: ServerGoalHandle):
        """Execute the explore behavior"""
        _result = StartOrchardNavigation.Result()
        # drive forward until costmap data finds a row, then succeed the goal. Timeout after 10s if no row found
        start_time = self.get_clock().now()
        while self._msg_orchard_nav_status.nav_state == OrchardNavState.EXPLORE:
            if self.row_detected:
                self.info("Row detected during exploration! Succeeding explore goal.")
                break
            if (self.get_clock().now() - start_time).nanoseconds > 10 * 1e9:  # 10 second timeout
                self.fatal("Explore behavior timed out without detecting a row.")
                _result.success = False
                _result.message = "Explore behavior timed out without detecting a row."
                goal_handle.abort()
                return _result
            # drive forward
            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            cmd_vel.twist.linear.x = 0.2  # Move forward at 0.2 m/s
            cmd_vel.twist.angular.z = 0.0
            self._pub_diff_drive_cmd_vel.publish(cmd_vel)
            self.get_clock.sleep_for(rclpy.duration.Duration(seconds=0.1))  # Sleep to prevent busy loop
        
        _result.success = True
        _result.message = "Explore behavior completed successfully."
        goal_handle.succeed()
        return _result

    def _action_goal_cb_turn_row(self, goal_handle):
        self.get_logger().info("Received turn_row goal request")
        return GoalResponse.ACCEPT
    
    def _action_cancel_cb_turn_row(self, goal_handle):
        self.get_logger().info("Received turn_row goal cancel request")
        return CancelResponse.ACCEPT
    
    def _action_goal_cb_return_home(self, goal_handle):
        self.get_logger().info("Received return_home goal request")
        return GoalResponse.ACCEPT
    
    def _action_cancel_cb_return_home(self, goal_handle):
        self.get_logger().info("Received return_home goal cancel request")
        return CancelResponse.ACCEPT
    
    def _action_exec_cb_return_home(self, goal_handle: ServerGoalHandle):
        """Execute the return_home behavior"""
        _result = StartOrchardNavigation.Result()
        if self._msg_initial_start_pose is None:
            self.fatal("No initial start pose received, cannot return home!")
            _result.success = False
            _result.message = "No initial start pose received, cannot return home!"
            goal_handle.abort()
            return _result
        
        self._pub_goal.publish(self._msg_initial_start_pose)
        
        while self._msg_orchard_nav_status.nav_state == OrchardNavState.RETURN_HOME:
            if self.check_if_goal_reached():
                self.info("Successfully returned home! Succeeding return_home goal.")
                break
            # Publish the initial start pose as the goal for the robot to navigate to
            self._pub_goal.publish(self._msg_initial_start_pose)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))  # Sleep to prevent busy loop
        
        _result.success = True
        _result.message = "Return home behavior completed successfully (not actually implemented)."
        goal_handle.succeed()
        return _result
    
    def _action_exec_cb_turn_row(self, goal_handle: ServerGoalHandle):
        """Execute the turn_row behavior"""
        _result = StartOrchardNavigation.Result()
        # turn in place until facing down the next row, then succeed the goal. Timeout after 10s if not facing row
        start_time = self.get_clock().now()
        while self._msg_orchard_nav_status.nav_state == OrchardNavState.TURN_ROW:
            if self.row_detected and self.robot_facing is not None and abs(self.robot_facing) < np.pi / 4:
                self.info("Successfully turned into the next row! Succeeding turn_row goal.")
                break
            if (self.get_clock().now() - start_time).nanoseconds > 10 * 1e9:  # 10 second timeout
                self.fatal("Turn row behavior timed out without successfully turning.")
                _result.success = False
                _result.message = "Turn row behavior timed out without successfully turning."
                goal_handle.abort()
                return _result
            
        _result.success = True
        _result.message = "Turn row behavior completed successfully."
        goal_handle.succeed()
        return _result
    
    def _action_goal_cb_traverse_row(self, goal_handle):
        self.get_logger().info("Received navigation goal request")
        return GoalResponse.ACCEPT

    def _action_cancel_cb_traverse_row(self, goal_handle):
        self.get_logger().info("Received navigation goal cancel request")
        return CancelResponse.ACCEPT

    def _action_exec_cb_traverse_row(self, goal_handle: ServerGoalHandle):
        """Get the goal"""
        self.goal_handle = goal_handle
        _result = StartOrchardNavigation.Result()

        while self._msg_orchard_nav_active_state.nav_state == OrchardNavState.TRAVERSE_ROW.value:
            # goal_reached = self.check_if_goal_reached()
            # if self.check_if_goal_reached():
            #     self.get_logger().info("Goal reached! Choosing a new goal...")
            #     self.choose_goal()
            with self._lock_local_costmap:
                local_costmap = self._msg_local_costmap
            with self._lock_global_costmap:
                global_costmap = self._msg_global_costmap
            
        
            obstacle_positions = self.get_obstacle_positions(costmap=local_costmap)
            rows = ochr.detect_orchard_rows(
                obstacle_positions=obstacle_positions,
                inlier_threshold=0.2,  # m distance threshold for inliers
                min_inliers=10,       # Minimum 10 inliers to consider a valid row
                max_iterations=100,   # RANSAC iterations
                min_row_separation=1.5  # Minimum distance between rows
            )
            # self.warn(rows)
            self.publish_row_markers(rows=rows)
            # self.warn(tree_positions)
            # if tree_positions:
            #     goal = self.choose_goal()
            #     self.info(f"{goal}")

            in_row = self.in_row(local_costmap.data)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))



        # if turn, publish next state as turn. else, go home
        # self._pub_orchard_nav_next_state.publish(OrchardNavStateMsg(nav_state=OrchardNavState.TURN_ROW.value))


        _result.success = True
        _result.message = ""
        goal_handle.succeed(_result)
        return

    def new_goal(self, x, y):
        """Publish a new goal pose for the robot to navigate to"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0  # Facing forward

        self._pub_goal.publish(goal_pose)
        self.goal_pose = goal_pose
        return

    def check_if_goal_reached(self):
        """Check if the robot has reached its current goal"""
        if self.goal_pose is None or self.robot_pose is None:
            return False

        distance = np.sqrt(
            (self.goal_pose.pose.position.x - self.robot_pose.position.x) ** 2
            + (self.goal_pose.pose.position.y - self.robot_pose.position.y) ** 2
        )
        return distance < 0.2  # Consider goal reached if within 20 cm

    def get_obstacle_positions(
        self,
        costmap,
        occupancy_threshold: int = 65,
        voxel_size: float = 0.1,
    ) -> list[tuple[float, float]]:
        """
        Extract occupied cell positions from a Nav2 costmap and return them
        in map frame coordinates, downsampled using a voxel grid filter.

        :param costmap: nav2_msgs/Costmap or OccupancyGrid message
        :param occupancy_threshold: cells at or above this value are considered obstacles.
            Nav2 costmap uses 0-254 scale (65 ~ inscribed, 253 = inscribed radius, 254 = lethal).
            Raw OccupancyGrid uses 0-100.
        :param voxel_size: spatial bin size (metres) for voxel grid downsampling.
            One point is kept per voxel, giving even spatial coverage regardless of
            cluster density. Should be >= costmap resolution.
        :returns: list of (x, y) tuples in map frame (metres)
        """
        if costmap is None or self.robot_pose is None:
            return []

        info = costmap.info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        resolution = info.resolution
        width = info.width
        height = info.height

        data = np.array(costmap.data, dtype=np.int16).reshape((height, width))
        rows, cols = np.where(data >= occupancy_threshold)

        # Convert to map frame
        map_x = origin_x + (cols + 0.5) * resolution
        map_y = origin_y + (rows + 0.5) * resolution
        points = np.column_stack((map_x, map_y))

        # Voxel grid: assign each point to a bin, keep one point per bin
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)

        return [tuple(p) for p in points[unique_indices]]

    def _get_neighbors(self, idx, width, map_data):
        """Get indices of 4-connected neighbors"""
        neighbors = []
        if idx + 1 < len(map_data) and (idx + 1) % width != 0:
            neighbors.append(idx + 1)
        if idx - 1 >= 0 and idx % width != 0:
            neighbors.append(idx - 1)
        neighbors.append(idx + width)
        neighbors.append(idx - width)
        return neighbors

    def in_row(self, map_data):
        if map_data is None:
            return False
        return any(cell > 50 for cell in map_data)

    def choose_goal(self):
        """Logic to choose a new goal based on sensor data and map"""
        return

    def publish_row_markers(
        self,
        rows: list,
        line_half_length: float = 4.0,
        max_residual: float = 0.5,
        lifetime_sec: float = 1.0,
    ):
        """
        Publish RViz markers for detected orchard rows. Each row produces three
        markers: a line strip showing the fitted row, an arrow showing direction,
        and a cylinder whose diameter encodes the fit residual.

        :param rows: list of OrchardRow from detect_orchard_rows()
        :param marker_pub: rclpy Publisher for visualization_msgs/MarkerArray
        :param line_half_length: metres to extend the line either side of the row centroid
        :param max_residual: residual (metres) at which the color saturates to full red
        :param lifetime_sec: marker lifetime in seconds; set to 0 for persistent
        """
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        lifetime = Duration(seconds=1.0).to_msg()


        # Delete all previous markers in the namespace before redrawing
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        for row_idx, row in enumerate(rows):
            color = self._residual_color(row.residual, max_residual)
            base_id = row_idx * 3  # 3 markers per row: 0=line, 1=arrow, 2=cylinder

            p = row.point        # (2,) centroid
            d = row.direction    # (2,) unit vector

            # --- LINE STRIP: fitted row axis ---
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = stamp
            line_marker.ns = "orchard_rows"
            line_marker.id = base_id
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.lifetime = lifetime
            line_marker.scale.x = 0.05  # line width (metres)
            line_marker.color = color

            start = Point(x=float(p[0] - d[0] * line_half_length),
                        y=float(p[1] - d[1] * line_half_length), z=0.0)
            end   = Point(x=float(p[0] + d[0] * line_half_length),
                        y=float(p[1] + d[1] * line_half_length), z=0.0)
            line_marker.points = [start, end]
            marker_array.markers.append(line_marker)

            # # --- ARROW: direction indicator at centroid ---
            # arrow_marker = Marker()
            # arrow_marker.header.frame_id = "map"
            # arrow_marker.header.stamp = stamp
            # arrow_marker.ns = "orchard_rows"
            # arrow_marker.id = base_id + 1
            # arrow_marker.type = Marker.ARROW
            # arrow_marker.action = Marker.ADD
            # arrow_marker.lifetime = lifetime
            # arrow_marker.scale.x = 0.6   # shaft length
            # arrow_marker.scale.y = 0.1   # shaft diameter
            # arrow_marker.scale.z = 0.15  # head diameter
            # arrow_marker.color = color

            # arrow_tail = Point(x=float(p[0]), y=float(p[1]), z=0.05)
            # arrow_head = Point(x=float(p[0] + d[0] * 0.6),
            #                 y=float(p[1] + d[1] * 0.6), z=0.05)
            # arrow_marker.points = [arrow_tail, arrow_head]
            # marker_array.markers.append(arrow_marker)

            # # --- CYLINDER: residual indicator at centroid ---
            # cyl_marker = Marker()
            # cyl_marker.header.frame_id = "map"
            # cyl_marker.header.stamp = stamp
            # cyl_marker.ns = "orchard_rows"
            # cyl_marker.id = base_id + 2
            # cyl_marker.type = Marker.CYLINDER
            # cyl_marker.action = Marker.ADD
            # cyl_marker.lifetime = lifetime
            # diameter = float(np.clip(row.residual * 4.0, 0.05, 1.0))  # scale for visibility
            # cyl_marker.scale.x = diameter
            # cyl_marker.scale.y = diameter
            # cyl_marker.scale.z = 0.1    # flat disc
            # cyl_marker.pose.position.x = float(p[0])
            # cyl_marker.pose.position.y = float(p[1])
            # cyl_marker.pose.position.z = 0.0
            # cyl_marker.pose.orientation.w = 1.0
            # cyl_marker.color = color
            # marker_array.markers.append(cyl_marker)

        self._pub_detected_row_markers.publish(marker_array)
        return
    
    def _residual_color(self, residual: float, max_residual: float = 0.5) -> ColorRGBA:
        """
        Map residual to a green -> yellow -> red color scale.

        :param residual: mean perpendicular inlier distance (metres)
        :param max_residual: residual at which color saturates to full red
        :returns: ColorRGBA with alpha=1
        """
        t = float(np.clip(residual / max_residual, 0.0, 1.0))
        c = ColorRGBA()
        c.a = 1.0

        if t < 0.5:
            # Green -> Yellow: ramp red up, keep green at 1
            c.r = t * 2.0
            c.g = 1.0
        else:
            # Yellow -> Red: ramp green down, keep red at 1
            c.r = 1.0
            c.g = (1.0 - t) * 2.0

        c.b = 0.0
        return c

def main(args=None):
    rclpy.init(args=args)
    node = OrchardNav()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()
    return
