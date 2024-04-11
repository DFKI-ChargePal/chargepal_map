""" This file implements the state >>Completion<< """
from __future__ import annotations

# libs
import rospy
from smach import State

from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig

# typing 
from typing import Any
from ur_pilot import Pilot


class Completion(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, outcomes=[out.stop])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo(f"Complete process in finale state.")
        self.pilot.disconnect()
        rospy.loginfo(f"Disconnect from robot.")
        outcome = out.stop
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(outcome, out.stop)
        return outcome
