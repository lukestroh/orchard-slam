import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose

from orchard_slam_bringup.logger_node import LoggerNode

import numpy as np


class OrchardNav(LoggerNode):
    def __init__(self):
        super().__init__("orchard_nav")

        # Subscriptions
        self._sub_nav_cmd_vel = self.create_subscription(
            msg_type=Twist, topic="/cmd_vel_nav", callback=self._sub_cb_nav_cmd_vel, qos_profile=10
        )

        # # # Subscriber for map (occupancy grid)
        self._sub_map = self.create_subscription(
            msg_type=OccupancyGrid, topic="/local_costmap/costmap", callback=self._sub_cb_map, qos_profile=10
        )
        self._sub_odom = self.create_subscription(
            msg_type=Odometry, topic="/odometry/filtered/local", callback=self._sub_cb_odom, qos_profile=10
        )

        # Publishers
        self._pub_diff_drive_cmd_vel = self.create_publisher(
            msg_type=TwistStamped, topic="/diff_drive_controller/cmd_vel", qos_profile=10
        )
        self._pub_goal = self.create_publisher(msg_type=PoseStamped, topic="/orchard_slam/goal_pose", qos_profile=10)

        # State variables
        self.goal_pose = None
        self._msg_laser_scan = None
        self._msg_map = None
        self._msg_robot_pose = None
        self.robot_facing = None
        self.row_detected = False
        print("Robot behavior node initialized.")
        return

    def _sub_cb_odom(self, msg: Odometry):
        """Callback for robot's current position (if needed)"""
        self._msg_robot_pose = msg
        robot_pose = self._msg_robot_pose.pose.pose
        self.robot_pose = robot_pose
        self.robot_facing = np.arctan2(self.robot_pose.orientation.z, self.robot_pose.orientation.w) * 2.0
        # self.get_logger().info(f"Current robot position: x={self.robot_pose.position.x}, y={self.robot_pose.position.y}")
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

    def _sub_cb_laser_scan(self, msg):
        """Callback for laser scan data (obstacle detection)"""
        self._msg_laser_scan = msg
        return

    def _sub_cb_map(self, msg):
        """Callback for occupancy grid map data"""
        self._msg_map = msg.data
        if self.check_if_goal_reached():
            self.get_logger().info("Goal reached! Choosing a new goal...")
            self.choose_goal()
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

    def get_tree_positions(self):
        """Identify tree positions by clustering occupied cells in the window map"""
        if self.window_map is None or self.robot_pose is None:
            return []

        tree_positions = []
        visited = set()
        grid_width = int(self.window_map.info.width)

        # Simple clustering: find connected occupied cells in window_map
        for idx, cell in enumerate(self.window_map.data):
            if cell > 50 and idx not in visited:  # Occupied cell
                # BFS to find connected cluster
                cluster = []
                queue = [idx]
                visited.add(idx)

                while queue:
                    current_idx = queue.pop(0)
                    cluster.append(current_idx)

                    # Check neighbors (4-connectivity)
                    for neighbor_idx in self._get_neighbors(current_idx, grid_width):
                        if (
                            neighbor_idx not in visited
                            and 0 <= neighbor_idx < len(self.window_map.data)
                            and self.window_map.data[neighbor_idx] > 50
                        ):
                            visited.add(neighbor_idx)
                            queue.append(neighbor_idx)

                # Calculate cluster centroid as tree position
                if cluster:
                    avg_x = sum(idx % grid_width for idx in cluster) / len(cluster)
                    avg_y = sum(idx // grid_width for idx in cluster) / len(cluster)
                    tree_positions.append((avg_x, avg_y))

        return tree_positions

    def _get_neighbors(self, idx, width):
        """Get indices of 4-connected neighbors"""
        neighbors = []
        if idx + 1 < len(self.map_data) and (idx + 1) % width != 0:
            neighbors.append(idx + 1)
        if idx - 1 >= 0 and idx % width != 0:
            neighbors.append(idx - 1)
        neighbors.append(idx + width)
        neighbors.append(idx - width)
        return neighbors

    def in_row(self):
        if self.map_data is None:
            return False
        return any(cell > 50 for cell in self.map_data)

    def choose_goal(self):
        """Logic to choose a new goal based on sensor data and map"""
        return


def main(args=None):
    rclpy.init(args=args)
    node = OrchardNav()
    rclpy.spin(node)
    rclpy.shutdown()
    return
