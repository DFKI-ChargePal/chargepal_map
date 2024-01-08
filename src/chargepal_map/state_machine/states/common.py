from __future__ import annotations

# global
import rospy
from smach import State

# local
import chargepal_map.state_machine.outcomes as out
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any


class Stop(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop])
        self.cfg = StateConfig(type(self), config=config)

    def execute(self, ud: Any) -> str:
        rospy.loginfo(f"Ended up in stop condition. Stop process...")
        return out.Common.stop
