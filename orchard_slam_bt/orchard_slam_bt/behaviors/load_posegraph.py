#!/usr/bin/env python3
import py_trees as pt
from orchard_slam_bringup.logger_node import LoggerNode
from rclpy.parameter import Parameter
from rclpy.task import Future

from geometry_msgs.msg import Pose2D
from slam_toolbox.srv import DeserializePoseGraph

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
    def __init__(self, name: str, map_name: str, match_type: int = DeserializePoseGraph.Request.START_AT_FIRST_NODE, initial_pose: tuple = (0.0, 0.0, 0.0)):
        super().__init__(name)
        self.name = name
        self.map_name = map_name
        self.match_type = match_type
        self.initial_pose = initial_pose
        return

    def setup(self, node: LoggerNode,) -> bool:
        self.node = node
        self.node.info(f"Setting up {self.name}")

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

        return True
    
    def initialise(self) -> None:
        """Call the LoadPosegraph service"""
        self.node.info(f"Requesting map load with name: {self.map_name}")

        map_name_abs_path = os.path.join(os.path.expanduser('~'), "orchard_slam_ws/src/orchard-slam/maps", self.map_name)

        load_map_req = DeserializePoseGraph.Request()
        load_map_req.filename = map_name_abs_path
        load_map_req.match_type = self.match_type
        
        # Set initial pose if provided
        
        pose = Pose2D()
        pose.x = self.initial_pose[0]
        pose.y = self.initial_pose[1]
        pose.theta = self.initial_pose[2]
        load_map_req.initial_pose = pose
        
        self._send_goal_future: Future = self._srv_client_deserialize_map.call_async(load_map_req)
        self._send_goal_future.add_done_callback(self._srv_cb_load_map)

        return
    
    def _srv_cb_load_map(self, future: Future):
        response: DeserializePoseGraph.Response = future.result()
        if response.result == DeserializePoseGraph.Response.RESULT_SUCCESS:
            self.goal_status = pt.common.Status.SUCCESS
            # Store the loaded map name on the blackboard for other behaviors to use
            self.blackboard.set("map_name", self.map_name)
        else:
            self.goal_status = pt.common.Status.FAILURE
        return