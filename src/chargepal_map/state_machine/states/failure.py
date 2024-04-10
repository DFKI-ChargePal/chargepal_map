""" This file implements the state >> Failure << """
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


class Failure(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, outcomes=[out.stop], input_keys=['job_id'], output_keys=['job_id'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo(f"Failure in manipulation action process.")
        job_id = ud.job_id

        return out.stop

