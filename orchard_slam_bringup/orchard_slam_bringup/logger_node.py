#!/usr/bin/env python3
# from rcl_interfaces.srv import ListParameters
# from ros2cli.node.direct import DirectNode
# from ros2param.api import call_get_parameters, call_list_parameters

import py_trees.console as con

from orchard_slam_bringup.base_node import BaseNode

# For writing param dumper:
# https://github.com/ros2/ros2cli/blob/master/ros2param/ros2param/verb/dump.py

class LoggerNode(BaseNode):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        # Loggers
        self.info = lambda x: self.get_logger().info(con.reset + con.green + f"\n{x}" + con.reset)
        self.debug = lambda x: self.get_logger().info(con.reset + con.white + f"\n{x}" + con.reset)
        self.warn = lambda x: self.get_logger().warn(con.reset + con.yellow + f"\n{x}" + con.reset)
        self.error = lambda x: self.get_logger().error(con.reset + con.red + f"\n{x}" + con.reset)
        self.fatal = lambda x: self.get_logger().fatal(con.reset + con.bold_red + f"\n{x}" + con.reset)
        return