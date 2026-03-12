import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import math
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


class RobotBehavior(Node):
    def __init__(self):
        super().__init__('robot_behavior')
        
        # # Publisher for robot movement (cmd_vel)
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)

        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10) 
        
        # # Subscriber for LaserScan (front-facing sensor)
        # self.laser_scan_subscriber = self.create_subscription(
        #     LaserScan, '/scan', self.laser_scan_callback, 10)
        
        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel_nav', self.cmd_vel_callback, 10)
        
        # # # Subscriber for map (occupancy grid)
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.map_callback, 10)
        
        
        self.position_subscriber = self.create_subscription(
            Odometry, '/odometry/filtered/local', self.position_callback, 10)
    
        
        # Timer to control robot behavior every 0.5 seconds
        # self.timer = self.create_timer(0.5, self.control_robot)
        
        # State variables
        self.goal_pose = None
        self.laser_scan_data = None
        self.map_data = None
        self.robot_pose = None
        self.robot_facing = None
        self.row_detected = False
        print("Robot behavior node initialized.")

    def check_if_goal_reached(self):
        """Check if the robot has reached its current goal"""
        if self.goal_pose is None or self.robot_pose is None:
            return False
        
        distance = math.sqrt(
            (self.goal_pose.pose.position.x - self.robot_pose.position.x) ** 2 +
            (self.goal_pose.pose.position.y - self.robot_pose.position.y) ** 2
        )
        return distance < 0.2  # Consider goal reached if within 20 cm

    def position_callback(self, msg):
        """Callback for robot's current position (if needed)"""
        self.robot_pose = msg.pose.pose
        self.robot_facing = math.atan2(
            self.robot_pose.orientation.z, self.robot_pose.orientation.w) * 2.0
        # self.get_logger().info(f"Current robot position: x={self.robot_pose.position.x}, y={self.robot_pose.position.y}")

    def cmd_vel_callback(self, msg):
        """Callback for cmd_vel to track robot's current velocity (if needed)"""
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = Twist()
        twist_stamped.twist.linear.x = msg.linear.x
        twist_stamped.twist.angular.z = msg.angular.z
        self.publisher_.publish(twist_stamped)  # Forward the cmd_vel message to the robot
        # self.get_logger().info(f"Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}")
        pass

    def laser_scan_callback(self, msg):
        """Callback for laser scan data (obstacle detection)"""
        self.laser_scan_data = msg.ranges

    def map_callback(self, msg):
        """Callback for occupancy grid map data"""
        self.map_data = msg.data
        if self.check_if_goal_reached():
            self.get_logger().info("Goal reached! Choosing a new goal...")
            self.choose_goal()


    def new_goal(self, x, y):
        """Publish a new goal pose for the robot to navigate to"""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0  # Facing forward
        self.pub_goal.publish(goal_msg)
        self.get_logger().info(f"Published new goal: x={x}, y={y}")

    def tree_postion(self):
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
                        if (neighbor_idx not in visited and 
                            0 <= neighbor_idx < len(self.window_map.data) and 
                            self.window_map.data[neighbor_idx] > 50):
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
        


def main(args=None):
    rclpy.init(args=args)
    robot_behavior = RobotBehavior()
    rclpy.spin(robot_behavior)
    rclpy.shutdown()


if __name__ == '__main__':
    main()