#!/usr/bin/env python3
from enum import Enum

"""NOTE: Be sure to update the OrchardNavStatus message definition if you add/remove states here"""


class OrchardNavState(Enum):
    IDLING = 0
    STOPPING = 1
    EXPLORING = 2
    TRAVERSING = 3
    TURNING = 4
    HOMING = 5
    ERROR = 6


VALID_TRANSITIONS = {
    OrchardNavState.IDLING: {
        OrchardNavState.EXPLORING,
        OrchardNavState.TRAVERSING,
        OrchardNavState.TURNING,
        OrchardNavState.HOMING,
        OrchardNavState.ERROR,
    },
    OrchardNavState.STOPPING: {
        OrchardNavState.ERROR,
        OrchardNavState.IDLING,
    },
    OrchardNavState.EXPLORING: {
        OrchardNavState.STOPPING,
         OrchardNavState.TRAVERSING,
         OrchardNavState.TURNING,
         OrchardNavState.HOMING,
    },
    OrchardNavState.TRAVERSING: {
        OrchardNavState.STOPPING,
        OrchardNavState.EXPLORING,
        OrchardNavState.TURNING,
        OrchardNavState.HOMING,
    },
    OrchardNavState.TURNING: {
        OrchardNavState.STOPPING,
        OrchardNavState.EXPLORING,
        OrchardNavState.TRAVERSING,
        OrchardNavState.HOMING,
    },
    OrchardNavState.HOMING: {
        OrchardNavState.STOPPING,
    },
    OrchardNavState.ERROR: {
        OrchardNavState.IDLING,
    },
}