from __future__ import annotations

# global
import rospy
from smach import State

# local
import chargepal_map.state_machine.outcomes as out
from chargepal_map.ui.user_client import UserClient
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any
from ur_pilot import Pilot


class Start(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[
            out.Common.stop, 
            out.ConnectToCarTwist.arm_in_driving_pose,
            out.ConnectToCarTwist.arm_in_bat_pre_obs,
            out.DisconnectFromCarTwist.arm_in_driving_pose,
            out.DisconnectFromCarTwist.arm_in_car_pre_obs
            ])

    def execute(self, ud: Any) -> str:
        rospy.loginfo(f"Start process...")
        # TODO: Make checks to observe current state
        
        return super().execute(ud)



class Stop(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop])

    def execute(self, ud: Any) -> str:
        rospy.loginfo(f"Ended up in stop condition. Stop process...")
        return out.Common.stop
