""" This file implements the state >>Stop<< """
from __future__ import annotations

# libs
import rospy
from smach import State

from chargepal_map.state_machine import outcomes as out
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.state_config import StateConfig

# typing 
from typing import Any
from ur_pilot import Pilot


class Stop(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, outcomes=[out.job_stopped])

    def execute(self, ud: Any) -> str:
        self.pilot.disconnect()
        rospy.loginfo(f"Ended up in stop state. Stop process by user...")
        outcome = out.job_stopped
        return outcome
