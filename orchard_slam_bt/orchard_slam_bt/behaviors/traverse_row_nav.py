#!/usr/bin/env python3
import py_trees as pt
import py_trees_ros as ptr
from orchard_slam_bringup.logger_node import LoggerNode

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.parameter import Parameter
from rclpy.task import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from orchard_msgs.action import StartOrchardNavigation
from orchard_msgs.msg import OrchardNavState as OrchardNavStateMsg
from orchard_nav.nav_state import OrchardNavState
from std_msgs.msg import Bool

import os


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
        self.node._pub_orchard_nav_active_state.publish(OrchardNavStateMsg(nav_state=OrchardNavState.TRAVERSE_ROW.value))
        start_nav_req = StartOrchardNavigation.Goal()
        self._send_goal_future = self._action_client_traverse_row.send_goal_async(start_nav_req)
        self._send_goal_future.add_done_callback(callback=self._send_goal_cb)
        return
    
    def _send_goal_cb(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        result: StartOrchardNavigation.Result = future.result()
        if result.success:
            self.goal_status = True
        return

    def update(self) -> pt.common.Status:
        # normal status check
        if self.goal_status is not None:
            if self.goal_status:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        return pt.common.Status.RUNNING
