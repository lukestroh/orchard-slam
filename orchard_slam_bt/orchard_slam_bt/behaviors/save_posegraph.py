#!/usr/bin/env python3
import py_trees as pt
from orchard_slam_bringup.logger_node import LoggerNode
from rclpy.parameter import Parameter
from rclpy.task import Future

from slam_toolbox.srv import SerializePoseGraph

import os

"""
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/orchard_slam_ws/src/orchard-slam/maps/map_name'}"
"""


class SavePosegraphBehavior(pt.behaviour.Behaviour):
    def __init__(self, name: str, map_name: str):
        super().__init__(name)
        self.name = name
        self.map_name = map_name
        return

    def setup(
        self,
        node: LoggerNode,
    ) -> None:
        self.node = node
        self.node.info(f"Setting up {self.name}")

        # Service clients
        self._srv_client_serialize_map = self.node.create_client(
            srv_type=SerializePoseGraph, srv_name="slam_toolbox/serialize_map"
        )
        # self._srv_client_serialize_map.wait_for_service()

        # Behavior state
        self.goal_status = None
        self.blackboard = pt.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="map_name", access=pt.common.Access.WRITE)

        return

    def initialise(self) -> None:
        """Call the SavePosegraph service"""
        self.node.info(f"Requesting map save with name: {self.map_name}")

        map_name_abs_path = os.path.join(
            os.path.expanduser("~"), "orchard_slam_ws/src/orchard-slam/maps", self.map_name
        )

        save_map_req = SerializePoseGraph.Request()
        save_map_req.filename = map_name_abs_path
        self._send_goal_future: Future = self._srv_client_serialize_map.call_async(save_map_req)
        self._send_goal_future.add_done_callback(self._srv_cb_save_map)

        return

    def _srv_cb_save_map(self, future: Future):
        response: SerializePoseGraph.Response = future.result()
        if response.result == SerializePoseGraph.Response.RESULT_SUCCESS:
            self.goal_status = True
        else:
            self.goal_status = False
        return

    def update(self) -> pt.common.Status:
        if self.goal_status is not None:
            if self.goal_status:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        return pt.common.Status.RUNNING
