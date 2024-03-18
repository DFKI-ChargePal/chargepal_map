""" This file implements the state >>Start<< """
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


class Start(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.stop, out.arm_in_wrong_ws, out.arm_ready_do, out.arm_ready_no])

    def execute(self, ud: Any) -> str:
        rospy.loginfo(f"Start process...")
        # TODO: Make checks to observe current state
        return super().execute(ud)
