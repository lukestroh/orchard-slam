#!/usr/bin/env python3
import py_trees as pt
import py_trees_ros as ptr
from orchard_slam_bringup.logger_node import LoggerNode

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.parameter import Parameter
from rclpy.task import Future

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import os


class NavigateToPoseBehavior(pt.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.name = name
        return

    def setup(
        self,
        node: LoggerNode,
    ) -> None:
        self.node = node
        self.node.info(f"Setting up {self.name}")

        # subscriptions
        self._sub_orchard_slam_goal_pose = self.node.create_subscription(
            msg_type=PoseStamped, topic="/orchard_slam/goal_pose", callback=self._sub_cb_goal_pose, qos_profile=10
        )

        # Action clients
        self._action_client_navigate_to_pose = ActionClient(
            node=self.node,
            action_type=NavigateToPose,
            action_name="navigate_to_pose",
        )
        self._action_client_navigate_to_pose.wait_for_server()

        # Behavior state
        self.goal_status = None
        self.last_sent_goal = None
        self.cancelling_goal = False
        self._cancel_goal_future = None
        self.blackboard = pt.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="goal_pose", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="mapping_complete", access=pt.common.Access.WRITE)

        self.goal_handle = None
        return

    def _sub_cb_goal_pose(self, msg: PoseStamped):
        """Store the latest goal pose from the /orchard_slam/goal_pose topic on the blackboard for the behavior to use"""
        self.blackboard.set("goal_pose", msg)
        self.node.info(f"Received new goal pose: {msg}")
        return

    def initialise(self) -> None:
        """Call the NavigateToPose action with the goal pose from the blackboard"""
        goal_pose = self.blackboard.get("goal_pose")
        # if goal_pose is None:
        #     self.node.error("No goal pose set on blackboard!")
        #     self.goal_status = False
        #     return

        self.node.info(f"Sending navigation goal: {goal_pose}")

        # TODO: Need a check to see if goal is running already, and if so, whether to cancel it before sending a new one
        self.test_goal_pose = PoseStamped()
        self.test_goal_pose.header.frame_id = "map"
        self.test_goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        self.test_goal_pose.pose.position.x = 1.0
        self.test_goal_pose.pose.position.y = 1.0

        return

    def send_goal(self, goal_pose: PoseStamped):
        navigate_to_pose_goal = NavigateToPose.Goal()
        navigate_to_pose_goal.pose = goal_pose
        navigate_to_pose_goal.behavior_tree = (
            ""  # TODO: figure out how to specify a custom behavior tree for navigation if needed
        )
        self._send_goal_future: Future = self._action_client_navigate_to_pose.send_goal_async(navigate_to_pose_goal)
        self._send_goal_future.add_done_callback(self._action_cb_navigate_to_pose)
        return

    def _action_cb_navigate_to_pose(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.node.error("Navigation goal rejected!")
            self.goal_status = False
            return
        self.goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._result_cb_navigate_to_pose)
        return

    def _result_cb_navigate_to_pose(self, future: Future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.info("Navigation goal succeeded!")
            self.goal_status = True
        else:
            self.node.error(f"Navigation goal failed with status code {status}!")
            self.goal_status = False
        return

        # if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
        #     self.node.info("Navigation goal succeeded!")
        #     self.goal_status = True
        # elif goal_handle.status == GoalStatus.STATUS_CANCELED:
        #     self.node.warn("Navigation goal canceled!")
        #     self.goal_status = False
        # elif goal_handle.status == GoalStatus.STATUS_ABORTED:
        #     self.node.error("Navigation goal aborted!")
        #     self.goal_status = False
        # elif goal_handle.status == GoalStatus.STATUS_EXECUTING:
        #     return
        # elif goal_handle.status == GoalStatus.STATUS_CANCELING:
        #     self.node.warn("Navigation goal canceling...")
        #     return
        # elif goal_handle.status == GoalStatus.STATUS_ACCEPTED:
        #     return
        # else:
        #     self.node.error(f"Navigation goal failed with status code {goal_handle.status}!")
        #     self.goal_status = False
        # return

    def update(self) -> pt.common.Status:
        # normal status check
        if self.goal_status is not None:
            if self.goal_status:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE

        # cancelling status check
        if self.cancelling:
            if self._cancel_goal_future is not None and self._cancel_goal_future.done():
                cancel_result = self._cancel_goal_future.result()
                if cancel_result.return_code == GoalStatus.STATUS_SUCCEEDED:
                    self.node.info("Navigation goal canceled successfully!")
                else:
                    self.node.error(f"Failed to cancel navigation goal with return code {cancel_result.return_code}!")
                self.cancelling = False
                self.goal_handle = None
                self.goal_status = None
            return pt.common.Status.RUNNING

        if self.blackboard.get("goal_pose") != self.last_sent_goal:
            self.node.info(f"Goal pose changed, canceling and resending...")
            if self.goal_handle is not None:
                self._cancel_goal_future = self.goal_handle.cancel_goal_async()
                self._cancel_goal_future.add_done_callback(self._cb_cancel_goal)
                self.cancelling = True
                return pt.common.Status.RUNNING
            else:
                self.goal_handle = None
            self.goal_status = None
            self.send_goal(self.test_goal_pose)
            return pt.common.Status.RUNNING

        return pt.common.Status.RUNNING

    def terminate(self, new_status: pt.common.Status) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.cancelling = False
        self.goal_status = None
        return
