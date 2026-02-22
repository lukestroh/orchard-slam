#!/usr/bin/env python3
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from orchard_msgs.msg import Orchard, OrchardRow, Tree

from orchard_slam_bringup.tf_node import TFNode

import json


class OrchardLayoutNode(TFNode):
    def __init__(self):
        super().__init__("orchard_config_node")
        """
        # Orchard.msg
        # Metadata information
        uint64 orchard_seed # needed for generating unique orchards given the same parameters
        string orchard_name
        string tree_namespace
        string tree_type

        # orchard generation parameters
        int64 n_rows
        int64 total_num_trees
        float32 avg_trees_per_row 
        float32 trees_per_row_std
        float32 avg_tree_spacing # average spacing between trees in a row, in meters
        float32 tree_spacing_std
        float32 avg_row_deviation # how much the trees in a row deviate from a straight line, in meters
        float32 std_row_deviation
        float32 avg_row_spacing # average spacing between rows, in meters
        float32 row_spacing_std


        orchard_msgs/Row[] rows

        uint64 seed # unique seed given orchard parameters and orchard_seed
        """

        # Parameters
        _params = {
            "orchard_config": Parameter.Type.STRING
        }
        self.declare_parameter_dict(_params)
        orchard_config_str = self.get_param_value(key="orchard_config")
        self.orchard_config = json.loads(orchard_config_str)
        
        # QoS profiles
        self._qos_orchard_config = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # Publishers
        self._pub_orchard_config = self.create_publisher(
            msg_type=Orchard,
            topic="orchard_config",
            qos_profile=self._qos_orchard_config,
        )

        # One-shot timer to publish orchard config after startup
        self._timer_pub_orchard_config = self.create_timer(
            timer_period_sec=10.0,
            callback=self._timer_cb_pub_orchard_config,
        )

        return
    
    def _timer_cb_pub_orchard_config(self):
        orchard_msg = Orchard()
        orchard_msg.orchard_seed = self.orchard_config["orchard_seed"]
        orchard_msg.seed = self.orchard_config["seed"]
        orchard_msg.orchard_name = self.orchard_config["orchard_name"]
        orchard_msg.tree_namespace = self.orchard_config["tree_namespace"]
        orchard_msg.tree_type = self.orchard_config["tree_type"]

        orchard_msg.n_rows = self.orchard_config["n_rows"]
        orchard_msg.total_num_trees = self.orchard_config["total_num_trees"]
        orchard_msg.avg_trees_per_row = self.orchard_config["avg_trees_per_row"]
        orchard_msg.trees_per_row_std = self.orchard_config["trees_per_row_std"]
        orchard_msg.avg_tree_spacing = self.orchard_config["avg_tree_spacing"]
        orchard_msg.tree_spacing_std = self.orchard_config["tree_spacing_std"]
        orchard_msg.avg_row_deviation = self.orchard_config["avg_row_deviation"]
        orchard_msg.std_row_deviation = self.orchard_config["std_row_deviation"]
        orchard_msg.avg_row_spacing = self.orchard_config["avg_row_spacing"]
        orchard_msg.row_spacing_std = self.orchard_config["row_spacing_std"]

        # Convert rows to OrchardRow messages
        for row in self.orchard_config["rows"]:
            row_msg = OrchardRow()
            row_msg.row_id = row["row_id"]
            row_msg.n_trees = row["num_trees"]
            

            for tree in row["trees"]:
                tree_msg = Tree()
                tree_msg.tree_namespace = tree["tree_namespace"]
                tree_msg.type = tree["type"]
                tree_msg.tree_id = tree["tree_id"]
                row_msg.trees.append(tree_msg)

            orchard_msg.rows.append(row_msg)

        # Publish the message
        self._pub_orchard_config.publish(orchard_msg)

        self._timer_pub_orchard_config.cancel() # Only publish once
        return

    

def main():
    rclpy.init()
    node = OrchardLayoutNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    return