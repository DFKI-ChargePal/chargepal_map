from __future__ import annotations

# global
import rospy
from smach import State

# local
import chargepal_map.state_machine.outcomes as out
from chargepal_map.state_machine.states.base import BaseState

# typing
from typing import Any
from chargepal_map.state_machine.process import ProcessABC


class Stop(State):

    def __init__(self, config: dict[str, Any], process: ProcessABC):

        State.__init__(self, outcomes=[out.Common.stop])
        BaseState.__init__(self, config, process)

    def execute(self, ud: Any) -> str:
        rospy.loginfo(f"Ended up in stop condition. Stop process...")
        return out.Common.stop
