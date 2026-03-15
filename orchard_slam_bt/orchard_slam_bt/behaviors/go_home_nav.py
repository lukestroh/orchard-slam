#!/usr/bin/env python3
import py_trees as pt


class GoHomeNavigationBehavior(pt.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.name = name
        return

    def setup(self, node) -> bool:
        self.node = node
        self.node.info(f"Setting up {self.name}")

        self.blackboard = pt.blackboard.Client(name=self.name)
        self.blackboard.register_key(key="orchard_nav/state", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="goal_pose", access=pt.common.Access.WRITE)
        self.blackboard.register_key(key="initial_start_pose", access=pt.common.Access.READ)
        return True

    def initialise(self) -> None:
        """Call the NavigateToPose action with the goal pose from the blackboard"""
        return

    def update(self) -> pt.common.Status:
        return pt.common.Status.SUCCESS
