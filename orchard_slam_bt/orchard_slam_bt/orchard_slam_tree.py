#!/usr/bin/env python3
"""
This node begins a new mapping session and saves the posegraph at the end. It can also be used to load a previous posegraph and continue mapping.
"""
import py_trees as pt
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


from orchard_slam_bringup.logger_node import LoggerNode
from orchard_nav.nav_state import OrchardNavState
from orchard_slam_bt.behaviors.save_posegraph import SavePosegraphBehavior
from orchard_slam_bt.behaviors.load_posegraph import LoadPosegraphBehavior
from orchard_slam_bt.behaviors.start_bag_record import StartBagRecordBehavior
from orchard_slam_bt.behaviors.stop_bag_record import StopBagRecordBehavior
from orchard_slam_bt.behaviors.traverse_row_nav import TraverseRowNavigationBehavior
from orchard_slam_bt.behaviors.turn_row_nav import TurningNavigationBehavior
from orchard_slam_bt.behaviors.go_home_nav import GoHomeNavigationBehavior

from geometry_msgs.msg import PoseStamped
from orchard_msgs.msg import OrchardNavStatus
from sensor_msgs.msg import Imu, NavSatFix


import datetime as dt
import functools as ft
import os
import glob


class OrchardSlamTree(LoggerNode):
    def __init__(self, node_name):
        super().__init__(node_name)
        # Parameters
        _params = {
            "load_map": Parameter.Type.BOOL,
            "initial_start_pose": Parameter.Type.STRING,  # "(x,y,theta)" in map frame
            "map_name": Parameter.Type.STRING,
            "record_bag": Parameter.Type.BOOL,
        }
        self.declare_parameters(namespace="", parameters=_params.items())
        self.load_map = self.get_param_val("load_map")
        self.map_name = self.get_param_val("map_name")
        self.record_bag = self.get_param_val("record_bag")
        if self.get_param_val("initial_start_pose") == "":
            self.initial_start_pose = None
        else:
            self.initial_start_pose = tuple(map(float, self.get_param_val("initial_start_pose").strip("()").split(",")))

        # Callback groups
        self._cb_group_reentrant = ReentrantCallbackGroup()
        self._cb_group_orchard_nav_status = MutuallyExclusiveCallbackGroup()

        # qos profiles
        self._qos_imu = QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self._qos_orchard_nav_status = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._qos_initial_start_pose = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.save_map_name = self.get_param_val("map_name") + "_" + dt.datetime.now().strftime("%Y%m%d-%H%M%S")

        # Callback groups
        self._cb_group_reentrant = ReentrantCallbackGroup()

        # Subscribers
        self._sub_gps = self.create_subscription(
            msg_type=NavSatFix,
            topic="/gps/filtered",
            callback=self._sub_cb_gps,
            callback_group=self._cb_group_reentrant,
            qos_profile=10,
        )
        self._sub_imu = self.create_subscription(
            msg_type=Imu,
            topic="/imu/fixed",
            callback=self._sub_cb_imu,
            callback_group=self._cb_group_reentrant,
            qos_profile=self._qos_imu,
        )
        self._sub_orchard_nav_status = self.create_subscription(
            msg_type=OrchardNavStatus,
            topic="/orchard_nav/status",
            callback=self._sub_cb_orchard_nav_status,
            callback_group=self._cb_group_reentrant,
            qos_profile=self._qos_orchard_nav_status,
        )

        # Publishers
        self._pub_orchard_nav_status = self.create_publisher(
            msg_type=OrchardNavStatus,  # TODO: potentially change to int or enum for more complex status
            topic="/orchard_nav/status",
            qos_profile=self._qos_orchard_nav_status,
        )
        self._pub_initial_start_pose = self.create_publisher(
            msg_type=PoseStamped,
            topic="/orchard_nav/initial_start_pose",
            qos_profile=self._qos_initial_start_pose,
        )

        # # Timers
        # self._timer_pub_orchard_nav_status = self.create_timer(
        #     timer_period_sec=0.25,
        #     callback=self._timer_cb_pub_orchard_nav_status,
        #     callback_group=self._cb_group_orchard_nav_status,
        # )

        # Blackboard TODO: probably don't need to register all keys on blackboard, just do from behaviors
        self.blackboard = pt.blackboard.Client(name=node_name)
        self.blackboard.register_key("goal_pose", access=pt.common.Access.WRITE)
        self.blackboard.register_key("gps/filtered", access=pt.common.Access.WRITE)
        self.blackboard.register_key("imu", access=pt.common.Access.WRITE)
        self.blackboard.register_key("map_name", access=pt.common.Access.WRITE)
        self.blackboard.register_key("mapping_complete", access=pt.common.Access.WRITE)
        self.blackboard.register_key("orchard_nav/state", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="initial_start_pose", access=pt.common.Access.WRITE)
        self.blackboard.register_key("record_bag", access=pt.common.Access.WRITE)
        self.blackboard.set("goal_pose", None)
        self.blackboard.set("gps/filtered", None)
        self.blackboard.set("imu", None)
        self.blackboard.set("initial_start_pose", self.initial_start_pose)
        self.blackboard.set("record_bag", self.record_bag)
        self.blackboard.set("map_name", self.map_name)
        self.blackboard.set("mapping_complete", False)
        self.blackboard.set("orchard_nav/state", OrchardNavState.IDLE)

        # Behavior tree
        self.tree = self.create_behavior_tree()
        self.snapshot_visitor = pt.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(ft.partial(self.post_tick_handler, self.snapshot_visitor))
        self.tree.add_visitor(self.snapshot_visitor)
        self.last_tree_snapshot = None

        return

    def _sub_cb_gps(self, msg: NavSatFix):
        """Store the latest GPS reading on the blackboard for other behaviors to use"""
        self.blackboard.set("gps/filtered", msg)
        return

    def _sub_cb_imu(self, msg: Imu):
        """Store the latest IMU reading on the blackboard for other behaviors to use"""
        self.blackboard.set("imu", msg)
        return

    def _sub_cb_orchard_nav_status(self, msg: OrchardNavStatus):
        self.blackboard.set("orchard_nav/state", msg.nav_state)
        return

    def _timer_cb_pub_orchard_nav_status(self):
        """Publish the current orchard navigation state from the blackboard at a regular interval"""
        nav_status_msg = OrchardNavStatus()
        nav_status_msg.nav_state = self.blackboard.get("orchard_nav/state")
        self._pub_orchard_nav_status.publish(nav_status_msg)
        return

    def create_behavior_tree(self):
        # Behaviors
        load_pose_graph_behavior = LoadPosegraphBehavior(
            name="load_posegraph_behavior", map_name=self.map_name, initial_pose=self.initial_start_pose
        )
        start_bag_record_behavior = StartBagRecordBehavior(name="start_bag_record_behavior")
        stop_bag_record_behavior = StopBagRecordBehavior(name="stop_bag_record_behavior")
        save_posegraph_behavior = SavePosegraphBehavior(name="save_posegraph", map_name=self.save_map_name)
        traverse_row_behavior = TraverseRowNavigationBehavior(name="traverse_row")
        turn_behavior = TurningNavigationBehavior(name="turn_around_row")
        go_home_behavior = GoHomeNavigationBehavior(name="go_home")

        load_pose_graph_guard = pt.decorators.EternalGuard(
            name="load_pose_graph_guard",
            child=load_pose_graph_behavior,
            condition=lambda: self.load_map is True,
        )

        start_bag_record_guard = pt.decorators.EternalGuard(
            name="start_bag_record_guard",
            child=start_bag_record_behavior,
            condition=lambda: self.blackboard.get("record_bag") is True,
            blackboard_keys=["record_bag"],
        )
        stop_bag_record_guard = pt.decorators.EternalGuard(
            name="stop_bag_record_guard",
            child=stop_bag_record_behavior,
            condition=lambda: self.blackboard.get("record_bag") is True,
            blackboard_keys=["record_bag"],
        )

        row_guard = pt.decorators.EternalGuard(
            name="row_guard",
            child=traverse_row_behavior,
            condition=lambda: self.blackboard.get("orchard_nav/state") == OrchardNavState.TRAVERSE_ROW,
            blackboard_keys=["orchard_nav/state"],
        )
        turn_guard = pt.decorators.EternalGuard(
            name="turn_guard",
            child=turn_behavior,
            condition=lambda: self.blackboard.get("orchard_nav/state") == OrchardNavState.TURN_ROW,
            blackboard_keys=["orchard_nav/state"],
        )
        go_home_guard = pt.decorators.EternalGuard(
            name="go_home_guard",
            child=go_home_behavior,
            condition=lambda: self.blackboard.get("orchard_nav/state") == OrchardNavState.RETURN_HOME,
            blackboard_keys=["orchard_nav/state"],
        )  # Go home behavior should publish a 'mapping_complete' flag on the blackboard when done to save posegraph

        navigation_selector = pt.composites.Selector(
            name="navigation_selector",
            children=[row_guard, turn_guard, go_home_guard],
            memory=False,
        )
        navigation_guard = pt.decorators.EternalGuard(
            name="navigation_guard",
            child=navigation_selector,
            condition=lambda: self.blackboard.get("mapping_complete") is False
            and self.blackboard.get("orchard_nav/state") != OrchardNavState.TASK_COMPLETE,
            blackboard_keys=["mapping_complete", "orchard_nav/state"],
        )
        navigation_success_is_running = pt.decorators.SuccessIsRunning(
            name="navigation_success_is_running",
            child=navigation_guard,
        )

        save_posegraph_guard = pt.decorators.EternalGuard(
            name="save_posegraph_guard",
            child=save_posegraph_behavior,
            condition=lambda: self.blackboard.get("mapping_complete") is True,
            blackboard_keys=["mapping_complete"],
        )

        task_selector = pt.composites.Selector(
            name="task_selector",
            children=[
                load_pose_graph_guard,
                start_bag_record_guard,
                stop_bag_record_guard,
                navigation_success_is_running,
                save_posegraph_guard,
            ],
            memory=False,
        )

        root_sequence = pt.composites.Sequence(
            name="RunSlamTree",
            children=[
                task_selector,
            ],
            memory=False,
        )

        tree = pt.trees.BehaviourTree(root=root_sequence)
        tree.setup(
            timeout=pt.common.Duration.INFINITE,
            node=self,
        )

        self.description()
        return tree

    def description(self):
        """Print description about the program"""
        msg = "OrchardSlamTree"
        self.info(
            pt.console.green
            + 80 * "*"
            + pt.console.reset
            + "\n"
            + pt.console.green
            + "* "
            + pt.console.bold_white
            + msg.center(80)
            + pt.console.reset
            + "\n"
            + pt.console.green
            + 80 * "*"
            + pt.console.reset
        )
        return

    def post_tick_handler(self, snapshot_visitor: pt.visitors.SnapshotVisitor, behavior_tree: pt.trees.BehaviourTree):
        """Write the tree snapshot to the console."""
        # snapshot_visitor.
        # if self.get_clock().now() - self._last_log_time > Duration(seconds=0.005):
        #     self.info("\n")
        current_snapshot = {_id: status for _id, status in snapshot_visitor.visited.items()}

        # self.info(current_snapshot)

        # if self.get_clock().now() - self._last_log_time > Duration(seconds=1.0):
        if current_snapshot != self.last_tree_snapshot:
            self.info(
                pt.display.unicode_tree(
                    root=behavior_tree.root,
                    visited=snapshot_visitor.visited,
                    previously_visited=snapshot_visitor.previously_visited,
                    show_status=True,
                )
                + "\n"
                + pt.display.unicode_blackboard()
                # + self.filtered_blackboard_display(exclude_keys=["/poses"])
            )
            # self._last_log_time = self.get_clock().now()
            self.last_tree_snapshot = current_snapshot


def main(args=None):
    rclpy.init(args=args)
    node = OrchardSlamTree(node_name="orchard_slam_tree")
    node.tree.tick_tock(
        period_ms=5.0,
        stop_on_terminal_state=True,
    )
    try:
        while rclpy.ok() and node.tree.root.status not in [pt.common.Status.SUCCESS, pt.common.Status.FAILURE]:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Tree has reached terminal state
        if node.tree.root.status == pt.common.Status.SUCCESS:
            node.info("Behavior tree completed successfully.")
        elif node.tree.root.status == pt.common.Status.FAILURE:
            node.error("Behavior tree failed.")

    except KeyboardInterrupt:
        node.info("Keyboard interrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return
