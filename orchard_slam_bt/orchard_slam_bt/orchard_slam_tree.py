#!/usr/bin/env python3
"""
This node begins a new mapping session and saves the posegraph at the end. It can also be used to load a previous posegraph and continue mapping.
"""
import py_trees as pt
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile


from orchard_slam_bringup.logger_node import LoggerNode
from orchard_slam_bt.behaviors.save_posegraph import SavePosegraphBehavior
from orchard_slam_bt.behaviors.load_posegraph import LoadPosegraphBehavior
from orchard_slam_bt.behaviors.start_bag_record import StartBagRecordBehavior
from orchard_slam_bt.behaviors.stop_bag_record import StopBagRecordBehavior

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
            "map_name": Parameter.Type.STRING,
            "record_bag": Parameter.Type.BOOL,
        }
        self.declare_parameters(namespace="", parameters=_params.items())
        self.load_map = self.get_param_val("load_map")
        self.load_map_name = None

        # Callback groups
        self._cb_group_reentrant = ReentrantCallbackGroup()

        # qos profiles
        self._qos_imu = QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)

        # Get the latest map by datetime if loading a map
        if self.load_map:
            map_name = self.get_param_val("map_name")
            map_files = glob.glob(os.path.join(os.getcwd(), "src", "orchard-slam", "maps", f"{map_name}_*.posegraph"))
            if len(map_files) == 0:
                self.error(f"No map files found for map name {map_name}. Starting a new mapping session instead.")
                pass

            # Sort map files by datetime and get the latest one
            def extract_datetime(filename):
                stem = filename.rsplit('.', 1)[0]  # remove extension
                dt_str = stem.rsplit('_', 1)[-1]  # grab last segment after underscore
                return dt.datetime.strptime(dt_str, "%Y%m%d-%H%M%S")

            self.load_map_name = max(map_files, key=extract_datetime)            
            self.info(f"Loading map: {self.load_map_name}")

        self.save_map_name = self.get_param_val("map_name") + "_" + dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.record_bag = self.get_param_val("record_bag")

        # Callback groups
        self._cb_group_reentrant = ReentrantCallbackGroup()

        # Subscribers
        self._sub_gps = self.create_subscription(
            msg_type=NavSatFix,
            topic="/gps/filtered",
            callback=self._sub_cb_gps,
            callback_group=self._cb_group_reentrant,
            qos_profile=10
        )
        self._sub_imu = self.create_subscription(
            msg_type=Imu,
            topic="/imu",
            callback=self._sub_cb_imu,
            callback_group=self._cb_group_reentrant,
            qos_profile=self._qos_imu
        )

        # Blackboard
        self.blackboard = pt.blackboard.Client(name=node_name)
        self.blackboard.register_key("map_name", access=pt.common.Access.WRITE)
        self.blackboard.register_key("gps_filtered", access=pt.common.Access.WRITE)
        self.blackboard.register_key("imu", access=pt.common.Access.WRITE)
        self.blackboard.set("gps_filtered", None)
        self.blackboard.set("imu", None)

        # Behavior tree
        self.tree = self.create_behavior_tree()
        self.snapshot_visitor = pt.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(ft.partial(self.post_tick_handler, self.snapshot_visitor))
        self.tree.add_visitor(self.snapshot_visitor)
        self.last_tree_snapshot = None

        return
    
    def _sub_cb_gps(self, msg: NavSatFix):
        """Store the latest GPS reading on the blackboard for other behaviors to use"""
        self.blackboard.set("gps_filtered", msg)

    def _sub_cb_imu(self, msg: Imu):
        """Store the latest IMU reading on the blackboard for other behaviors to use"""
        self.blackboard.set("imu", msg)
        return
    
    def create_behavior_tree(self):
        # Behaviors
        while (self.blackboard.get("gps_filtered") is None or self.blackboard.get("imu") is None) and self.load_map:
            self.info("Waiting for GPS and IMU data on the blackboard before creating the behavior tree...")
            rclpy.spin_once(self, timeout_sec=0.1)
        load_pose_graph_behavior = LoadPosegraphBehavior(name="load_posegraph_behavior", load_map=self.load_map, map_name=self.load_map_name)
        # start_bag_record_behavior = StartBagRecordBehavior(name="start_bag_record_behavior", record_bag=self.record_bag)
        save_posegraph_behavior = SavePosegraphBehavior(name="save_posegraph_behavior", map_name=self.save_map_name)
        


        root_sequence = pt.composites.Sequence(
            name="RunSlamTree",
            children=[
                load_pose_graph_behavior,
                # start_bag_record_behavior,
                # save_posegraph_behavior
            ],
            memory=False,
        )

        root = root_sequence # edit for later when we have more behaviors

        tree = pt.trees.BehaviourTree(root=root)
        tree.setup(
            timeout=pt.common.Duration.INFINITE,
            node=self,
        )

        self.description()
        return tree
    
    def description(self):
        """Print description about the program"""
        msg = "RunSlamTree"
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
    
    def post_tick_handler(
        self, snapshot_visitor: pt.visitors.SnapshotVisitor, behavior_tree: pt.trees.BehaviourTree
    ):
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
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.info("Keyboard interrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return