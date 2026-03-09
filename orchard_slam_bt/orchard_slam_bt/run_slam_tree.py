#!/usr/bin/env python3
import py_trees as pt
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from orchard_slam_bringup.logger_node import LoggerNode

from orchard_slam_bt.behaviors.save_posegraph import SavePosegraphBehavior
from orchard_slam_bt.behaviors.load_posegraph import LoadPosegraphBehavior


class RunSlamTree(LoggerNode):
    def __init__(self, node_name):
        super().__init__(node_name)
        # Parameters
        _params = {
            "map_name": Parameter.Type.STRING,
        }
        self.declare_parameters(namespace="", parameters=_params.items())
        self.map_name = self.get_param_val("map_name")

        # Callback groups
        self._cb_group_reentrant = ReentrantCallbackGroup()

        # Subscribers

        # Blackboard
        self.blackboard = pt.blackboard.Client(name=node_name)
        self.blackboard.register_key("map_name", access=pt.common.Access.WRITE)

        # Behavior tree
        self.tree = self.create_behavior_tree()

        return
    
    def create_behavior_tree(self):
        # Behaviors
        save_posegraph_behavior = SavePosegraphBehavior(name="save_posegraph_behavior", map_name=self.map_name)

        root_sequence = pt.composites.Sequence(
            name="RunSlamTree",
            children=[
                save_posegraph_behavior
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
    

def main(args=None):
    rclpy.init(args=args)
    node = RunSlamTree(node_name="run_slam_tree")
    node.tree.tick_tock(period_ms=5.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.info("Keyboard interrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return