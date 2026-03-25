#!/usr/bin/env python3
"""This behavior manages the orchard nav state on the blackboard and the /orchard_nav/status topic."""
import py_trees as pt



class NavigationStateManagerBehavior(pt.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.name = name
        return

    def setup(self, node) -> bool:
        self.node = node
        self.node.info(f"Setting up {self.name}")

        # Publishers
        # self._pub_orchard_nav_status = self.node.create_publisher(
        #     msg_type=OrchardNavStatus,  # TODO: potentially change to int or enum for more complex status
        #     topic="/orchard_nav/status",
        #     qos_profile=self._qos_orchard_nav_status,
        # )

        # Subscribers
        # self._sub_orchard_nav_status = self.node.create_subscription(
        #     msg_type=OrchardNavStatus, --- IGNORE ---
        #     topic="/orchard_nav/status",
        #     callback=self._sub_cb_orchard_nav_status,
        #     callback_group=self._cb_group_reentrant,
        # )

        return True

    def initialise(self) -> None:
        """Publish the current orchard navigation state from the blackboard at a regular interval"""
        return

    def update(self) -> pt.common.Status:
        return pt.common.Status.SUCCESS

    def _sub_cb_orchard_nav_status(self, msg):
        """Callback for orchard navigation status updates"""
        self.blackboard.set("orchard_nav/state", msg.nav_state)
        return