""" This file implements the state >>Malfunction<< """
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


class Malfunction(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, outcomes=[out.job_failed])

    def execute(self, ud: Any) -> str:
        print(), rospy.logwarn(f"Malfunction during the process.")
        # self.pilot.disconnect()
        rospy.loginfo(f"Unable to recover the robot arm by itself.")
        outcome = out.job_failed
        return outcome
