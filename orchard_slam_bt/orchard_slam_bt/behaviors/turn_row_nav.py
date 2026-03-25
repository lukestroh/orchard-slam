#!/usr/bin/env python3
import py_trees as pt


class TurningNavigationBehavior(pt.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.name = name
        return

    def setup(self, node) -> None:
        self.node = node
        self.node.info(f"Setting up {self.name}")
        return

    def initialise(self) -> None:
        """Call the NavigateToPose action with the goal pose from the blackboard"""
        return

    def update(self) -> pt.common.Status:
        return pt.common.Status.SUCCESS
