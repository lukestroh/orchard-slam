#!/usr/bin/env python3
import py_trees as pt

from orchard_nav.nav_state import OrchardNavState


class OrchardStateManager(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="OrchardStateManager")
        self.blackboard = pt.blackboard.Client(name=self.name)
        self.blackboard.register_key("orchard_nav/state", access=pt.common.Access.WRITE)
        self.blackboard.register_key("current_row", access=pt.common.Access.WRITE)
        self.blackboard.register_key("total_rows", access=pt.common.Access.READ)
        self.blackboard.register_key("mapping_complete", access=pt.common.Access.WRITE)

    def update(self) -> pt.common.Status:
        current_state = self.blackboard.get("orchard_nav/state")

        if current_state == OrchardNavState.TRAVERSE_ROW:
            if self._at_row_end():
                self.blackboard.orchard_nav_state = OrchardNavState.TURN_ROW
        elif current_state == OrchardNavState.TURN_ROW:
            if self._turn_complete():
                next_row = self.blackboard.current_row + 1
                if next_row >= self.blackboard.total_rows:
                    self.blackboard.orchard_nav_state = OrchardNavState.RETURN_HOME
                else:
                    self.blackboard.current_row = next_row
                    self.blackboard.orchard_nav_state = OrchardNavState.TRAVERSE_ROW

        elif current_state == OrchardNavState.RETURN_HOME:
            if self._at_home():
                self.blackboard.mapping_complete = True
                self.blackboard.orchard_nav_state = OrchardNavState.TASK_COMPLETE

        return pt.common.Status.RUNNING  # never succeeds or fails
