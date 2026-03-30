#!/usr/bin/env python3


import py_trees as pt


class IdleBehavior(pt.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.name = name
        return

    def setup(self, node):
        self.node = node
        self.node.info(f"Setting up {self.name}")
        return

    def initialise(self):
        # Somehow need to make sure that the robot doesn't move while in this state, without giving behavior access to
        # the ros side of things. How do I efficiently call a "stop" service once when entering this state, 
        # without having to call it every tick?
        #  
        return

    def update(self):
        return pt.common.Status.SUCCESS
