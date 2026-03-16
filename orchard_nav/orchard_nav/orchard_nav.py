import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from orchard_msgs.action import StartOrchardNavigation
from orchard_msgs.msg import OrchardNavState as OrchardNavStateMsg
from geometry_msgs.msg import Pose, PoseStamped

from orchard_slam_bringup.logger_node import LoggerNode
from orchard_nav.nav_state import OrchardNavState

import numpy as np
from threading import Lock


class OrchardNav(LoggerNode):
    def __init__(self):
        super().__init__("orchard_nav")

        # locks
        self._lock_local_costmap = Lock()

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
        
            tree_positions = self.get_tree_positions(local_costmap)
            # self.warn(tree_positions)
            # if tree_positions:
            #     goal = self.choose_goal()
            #     self.info(f"{goal}")

            in_row = self.in_row(local_costmap.data)
            self.warn(f"in_row: {in_row}")



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

    def get_tree_positions(self, local_costmap):
        """Identify tree positions by clustering occupied cells in the window map"""

        if local_costmap is None or self.robot_pose is None:
            return []
        
        tree_positions = []
        visited = set()
        grid_width = int(local_costmap.info.width)

        # Simple clustering: find connected occupied cells in window_map
        for idx, cell in enumerate(local_costmap.data):
            if cell > 50 and idx not in visited:  # Occupied cell
                # BFS to find connected cluster
                cluster = []
                queue = [idx]
                visited.add(idx)

                while queue:
                    current_idx = queue.pop(0)
                    cluster.append(current_idx)

                    # Check neighbors (4-connectivity)
                    for neighbor_idx in self._get_neighbors(current_idx, grid_width, local_costmap.data):
                        if (
                            neighbor_idx not in visited
                            and 0 <= neighbor_idx < len(local_costmap.data)
                            and local_costmap.data[neighbor_idx] > 50
                        ):
                            visited.add(neighbor_idx)
                            queue.append(neighbor_idx)

                # Calculate cluster centroid as tree position
                if cluster:
                    avg_x = sum(idx % grid_width for idx in cluster) / len(cluster)
                    avg_y = sum(idx // grid_width for idx in cluster) / len(cluster)
                    tree_positions.append((avg_x, avg_y))

        return tree_positions

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


def main(args=None):
    rclpy.init(args=args)
    node = OrchardNav()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()
    return
