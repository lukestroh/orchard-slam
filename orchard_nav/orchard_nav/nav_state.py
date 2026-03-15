#!/usr/bin/env python3
from enum import Enum

"""NOTE: Be sure to update the OrchardNavStatus message definition if you add/remove states here"""
class OrchardNavState(Enum):
    IDLE = 1
    EXPLORING = 2
    TRAVERSE_ROW = 3
    TURN_ROW = 4
    RETURN_HOME = 5
    TASK_COMPLETE = 6
