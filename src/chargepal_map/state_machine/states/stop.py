""" This file implements the state >>Stop<< """
from __future__ import annotations

# libs
import rospy
from smach import State

from chargepal_map.ui.user_client import UserClient
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig

# typing 
from typing import Any
from ur_pilot import Pilot


class Stop(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.stop])

    def execute(self, ud: Any) -> str:
        rospy.loginfo(f"Ended up in stop condition. Stop process...")
        return out.stop
