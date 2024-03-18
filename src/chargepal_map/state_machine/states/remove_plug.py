""" This file implements the state >>RemovePlug<< """

from __future__ import annotations
# libs
import rospy
import numpy as np
from smach import State

from chargepal_map.ui.user_client import UserClient
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any
from ur_pilot import Pilot


class RemovePlug(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.stop, out.plug_removed_do, out.plug_removed_no])

    def execute(self, user_data: Any) -> str:
        print(), rospy.loginfo('Start removing the plug from battery')
        plug_out_ft = np.array([0.0, 0.0, -abs(self.cfg.data['force']), 0.0, 0.0, 0.0])
        with self.pilot.context.force_control():
            success = self.pilot.tcp_force_mode(
                wrench=plug_out_ft,
                compliant_axes=[0, 0, 1, 0, 0, 0],
                distance=self.cfg.data['moving_distance'],
                time_out=self.cfg.data['force_mode_time_out'])
        if not success:
            raise RuntimeError(f"Error while trying to unplug. Plug is probably still connected.")
        rospy.loginfo(f"Plug successfully removed from the battery socket.")
        return self.uc.request_action(out.plug_removed, out.stop)
