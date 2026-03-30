#!/usr/bin/env python3
import os

import py_trees as pt
import py_trees_ros as ptr
from geometry_msgs.msg import PoseStamped
from orchard_msgs.action import StartOrchardNavigation
from orchard_msgs.action._start_orchard_navigation import (
    StartOrchardNavigation_GetResult_Response,
)
from orchard_msgs.msg import OrchardNavState as OrchardNavStateMsg
from orchard_nav.nav_state import OrchardNavState
from orchard_slam_bringup.logger_node import LoggerNode
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Bool


class TraverseRowNavigationBehavior(pt.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.name = name
        return

    def setup(self, node: LoggerNode) -> None:
        self.node = node
        self.node.info(f"Setting up {self.name}")

        # Action clients
        self._action_client_traverse_row = ActionClient(
            node=self.node,
            action_type=StartOrchardNavigation,
            action_name="/orchard_nav/traverse_row",
        )
        self._action_client_traverse_row.wait_for_server()

        # Behavior state
        self.goal_status = None
        self.last_sent_goal = None
        self.blackboard = pt.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="goal_pose", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="mapping_complete", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="orchard_nav/state", access=pt.common.Access.WRITE)

        self.goal_handle = None
        return

    def initialise(self) -> None:
        """Call the NavigateToPose action with the goal pose from the blackboard"""
        start_nav_req = StartOrchardNavigation.Goal()
        self._send_goal_future = self._action_client_traverse_row.send_goal_async(start_nav_req)
        self._send_goal_future.add_done_callback(self._send_goal_cb)
        return

    def _send_goal_cb(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()  # type: ignore[assignment]
        if not goal_handle.accepted:
            self.node.error(f"{self.name}: Goal rejected")
            self.goal_status = False
            return
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._goal_response_cb)
        return

    def _goal_response_cb(self, future: Future):
        goal_resp = future.result()
        assert goal_resp is not None
        result = goal_resp.result
        status = goal_resp.status

        self.node.info(f"Resp type: {type(goal_resp)}, status: {status}, result: {result}")

        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.success:
                self.node.info(f"{self.name}: Goal succeeded")
                self.goal_status = True
                self.blackboard.set("orchard_nav/state", result.next_state)
            else:
                self.node.error(f"{self.name}: Goal failed.")
                self.goal_status = False
                self.blackboard.set("orchard_nav/state", OrchardNavState.ERROR)
        else:
            self.node.error(f"{self.name}: Goal failed with status {status}")
            self.goal_status = False

        return

    def update(self) -> pt.common.Status:
        if self.goal_status is not None:
            if self.goal_status:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        return pt.common.Status.RUNNING
