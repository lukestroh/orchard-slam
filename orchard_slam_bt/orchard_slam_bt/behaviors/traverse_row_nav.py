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
        self._navigate_to_pose_client = ActionClient(
            node=self.node,
            action_type=StartOrchardNavigation,
            action_name="/orchard_nav/start_in_row_navigation",
        )

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
        goal_pose = self.blackboard.get("goal_pose")
        # if goal_pose is None:
        #     self.node.error("No goal pose set on blackboard!")
        #     self.goal_status = False
        #     return

        self.node.info(f"Sending navigation goal: {goal_pose}")

        self.test_goal_pose = PoseStamped()
        self.test_goal_pose.header.frame_id = "map"
        self.test_goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        self.test_goal_pose.pose.position.x = 1.0
        self.test_goal_pose.pose.position.y = 1.0

        return

    def update(self) -> pt.common.Status:
        # normal status check
        if self.goal_status is not None:
            if self.goal_status:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        return pt.common.Status.RUNNING
