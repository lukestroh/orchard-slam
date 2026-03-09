#!/usr/bin/env python3
import py_trees as pt
from orchard_slam_bringup.logger_node import LoggerNode
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.task import Future

from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import NavSatFix
from slam_toolbox.srv import DeserializePoseGraph

import numpy as np
import os


"""
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{filename: '$HOME/orchard_slam_ws/src/orchard-slam/maps/map_name'}"


int8 UNSET = 0
int8 START_AT_FIRST_NODE = 1
int8 START_AT_GIVEN_POSE = 2
int8 LOCALIZE_AT_POSE = 3

# inital_pose should be Map -> base_frame (parameter, generally base_link)
#

string filename
int8 match_type
geometry_msgs/Pose2D initial_pose
---
"""


class LoadPosegraphBehavior(pt.behaviour.Behaviour):
    def __init__(self, name: str, load_map: bool, map_name: str, match_type: int = DeserializePoseGraph.Request.START_AT_GIVEN_POSE, initial_pose: tuple = None):
        super().__init__(name)
        self.name = name
        self.load_map = load_map
        self.map_name = map_name
        self.match_type = match_type
        self.initial_pose = initial_pose
        self.goal_status = None
        return

    def setup(self, node: LoggerNode,) -> bool:
        self.node = node
        self.node.info(f"Setting up {self.name}")
        if not self.load_map:
            self.goal_status = True
            return True

        # Service clients
        self._srv_client_deserialize_map = self.node.create_client(
            srv_type=DeserializePoseGraph,
            srv_name="slam_toolbox/deserialize_map"
        )
        self._srv_client_deserialize_map.wait_for_service()

        # Behavior state
        self.goal_status = None
        self.blackboard = pt.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="map_name", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="gps_filtered", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="imu", access=pt.common.Access.WRITE)

        return True
    
    def initialise(self) -> None:
        """Prepare for map loading — actual request is sent from update() once GPS is available"""
        self.node.info(f"Requesting map load with name: {self.map_name}")

        map_name_abs_path = os.path.join(os.path.expanduser('~'), "orchard_slam_ws/src/orchard-slam/maps", self.map_name)
        map_name_abs_path = map_name_abs_path.removesuffix('.posegraph')

        if self.initial_pose is None:
            gps_data: NavSatFix = self.blackboard.get("gps_filtered")
            imu_data = self.blackboard.get("imu")
            # Compute robot position in map frame: FROM map origin (0,0) TO robot's current lat/lon
            dx, dy = self.latlon_to_dxdy(0.0, 0.0, gps_data.latitude, gps_data.longitude)
            # Extract yaw from IMU quaternion (ROS ENU: radians from east, CCW)
            q = imu_data.orientation
            theta = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        else:
            dx, dy = self.initial_pose[0], self.initial_pose[1]
            theta = self.initial_pose[2]

        load_map_req = DeserializePoseGraph.Request()
        load_map_req.filename = map_name_abs_path
        load_map_req.match_type = self.match_type
        
        pose = Pose2D()
        pose.x = dx
        pose.y = dy
        pose.theta = theta
        load_map_req.initial_pose = pose
        
        self._send_goal_future: Future = self._srv_client_deserialize_map.call_async(load_map_req)
        self._send_goal_future.add_done_callback(self._srv_cb_load_map)
        return
    
    def _srv_cb_load_map(self, future: Future):
        response: DeserializePoseGraph.Response = future.result()
        self.goal_status = True
        # Store the loaded map name on the blackboard for other behaviors to use
        self.blackboard.set("map_name", self.map_name)

        return
    
    def latlon_to_dxdy(self, lat1, lon1, lat2, lon2):
        r_earth = 6371000 # radius of Earth in meters
        dy = r_earth * np.radians(lat2 - lat1)                          # north/south
        dx = r_earth * np.radians(lon2 - lon1) * np.cos(np.radians(lat1))  # east/west
        return dx, dy
    
    
    def update(self) -> pt.common.Status:
        if self.goal_status is not None:
            if self.goal_status:
                return pt.common.Status.SUCCESS
            else:                
                return pt.common.Status.FAILURE

        return pt.common.Status.RUNNING
        
    