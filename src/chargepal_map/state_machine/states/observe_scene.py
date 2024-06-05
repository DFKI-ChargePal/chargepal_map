""" This file implements the state >>ObserveScene<< """
from __future__ import annotations

# libs
import rospy
import ur_pilot
from smach import State
import spatialmath as sm

from chargepal_map.core import job_ids
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.utils import StateMachineError

# typing
from typing import Any
from ur_pilot import Pilot


class ObserveScene(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.stop, out.socket_obs], 
                       input_keys=['job_id'],
                       output_keys=['job_id', 'T_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start observing the scene')
