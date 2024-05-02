""" This file implements the state >>Start<< """
from __future__ import annotations
# libs
import rospy
from smach import State

from chargepal_map.core import job_ids
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.utils import StateMachineError

# typing
from typing import Any
from ur_pilot import Pilot


class StartPlugging(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self,
                       outcomes=[out.stop, out.arm_ready_to_plug_in, out.arm_ready_to_plug_out],
                       input_keys=['job_id'],
                       output_keys=['job_id'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo(f"Find proper plugging process.")
        job_id = ud.job_id
        if job_id in job_ids.plug_in():
            outcome = out.arm_ready_to_plug_in
        elif job_id in job_ids.plug_out():
            outcome = out.arm_ready_to_plug_out
        else:
            raise StateMachineError(f"Job ID '{job_id}' doesn't fit into plugging process.")
        rospy.loginfo(f"Start plugging process: {job_id}")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(outcome, out.stop)
        return outcome
