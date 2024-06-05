""" This file implements the state >>Completion<< """
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


class Completion(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, outcomes=[out.job_complete])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo(f"Complete process successfully in its finale state.")
        outcome = out.job_complete
        return outcome


class Incompletion(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, outcomes=[out.job_incomplete])

    def execute(self, ud: Any) -> str:
        print(), rospy.logwarn(f"Complete process unsuccessfully! However, the robot ended in a safe state.")
        outcome = out.job_incomplete
        return outcome
