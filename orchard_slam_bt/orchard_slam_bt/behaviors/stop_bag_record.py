#!/usr/bin/env python3
import py_trees as pt

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.task import Future
from rclpy.parameter import Parameter

from ros2bag_msgs.srv import StopRecord

from orchard_slam_bringup.logger_node import LoggerNode


class StopBagRecordBehavior(pt.behaviour.Behaviour):
    def __init__(self, name: str, record_bag: bool):
        super(StopBagRecordBehavior, self).__init__(name)
        self.name = name
        self.record_bag = record_bag
        return

    def setup(self, node: LoggerNode):
        self.node = node
        self.info(f"Setting up {self.name}")

        # Service clients
        self._srv_client_stop_bag_record = self.node.create_client(srv_name="/stop_bag_record", srv_type=StopRecord)
        self._srv_client_stop_bag_record.wait_for_service()

        # Behaviour attributes
        self.goal_status = None

        self.blackboard = pt.blackboard.Client(name=self.name)

        return

    def initialise(self):
        """Call the stop bag record service"""
        if self.record_bag == False:
            self.goal_status = True
        else:
            self.goal_status = None
        stop_record_req = StopRecord.Request()
        self._send_goal_future: Future = self._srv_client_stop_bag_record.call_async(request=stop_record_req)
        self._send_goal_future.add_done_callback(callback=self._send_goal_cb)
        return

    def _send_goal_cb(self, future: Future):
        result: StopRecord.Response = future.result()
        self.goal_status = result.success
        return

    def update(self):
        if self.goal_status is not None:
            if self.goal_status:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.FAILURE
        return pt.common.Status.RUNNING