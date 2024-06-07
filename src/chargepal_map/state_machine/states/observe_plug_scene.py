""" This file implements the state >>ObservePlugScene<< """
from __future__ import annotations
import re

# libs
import rospy
import ur_pilot
from smach import State
import spatialmath as sm

from chargepal_map.job import Job
from chargepal_map.state_machine import outcomes as out
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.utils import StateMachineError

# typing
from typing import Any
from ur_pilot import Pilot


class ObservePlugScene(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[
                           out.plug_scene_obs,
                           out.err_obs_plug_retry, 
                           out.err_obs_plug_recover,
                           out.job_stopped], 
                       input_keys=['job'],
                       output_keys=['job', 'T_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start observing the scene')
        outcome = out.err_scene_incomplete
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.plug_scene_obs, out.job_stopped)
        return outcome
