#!/usr/bin/env python3
from rclpy.node import Node

import os
import yaml


# For writing param dumper:
# https://github.com/ros2/ros2cli/blob/master/ros2param/ros2param/verb/dump.py

class BaseNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        return
    
    def declare_parameter_dict(self, **kwargs):
        for key, val in kwargs.items():
            self.declare_parameter(key, val)
        return

    def get_param_val(self, key):
        try:
            return self.get_parameter(key).value
        except Exception as ex:
            self.get_logger().error(ex)
            return None

    def dump_params(self, dirname):
        name = self.get_name()
        yaml_output = {name: {"ros__parameters": {}}}
        for key, val in self.get_parameters_by_prefix("").items():
            yaml_output[name]["ros__parameters"][key] = val.value

        with open(os.path.join(dirname, f"params_{name}.yaml"), "w") as outfile:
            yaml.dump(yaml_output, outfile, default_flow_style=False)
        return