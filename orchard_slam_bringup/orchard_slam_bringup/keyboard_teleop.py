#!/usr/bin/env python3

import rclpy
from rclpy.parameter import Parameter


from orchard_slam_bringup.logger_node import LoggerNode


class KeyboardTeleop(LoggerNode):
    def __init__(self):
        super().__init__("keyboard_teleop")
            
        return


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    return