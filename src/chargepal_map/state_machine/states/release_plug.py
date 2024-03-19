""" This file implements the state >>ReleasePlug<< """
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


class ReleasePlug(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.stop, out.plug_released], 
                       input_keys=['job_id'],
                       output_keys=['job_id'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start releasing the arm from the plug')
        with self.pilot.context.force_control():
            # Release plug via twisting end-effector
            success = self.pilot.screw_ee_force_mode(4.0, -np.pi / 2, 12.0)
            if not success:
                raise RuntimeError(f"Robot did not succeed in opening the twist lock. "
                                    f"Robot is probably in an undefined condition.")
            release_ft = np.array([0.0, 0.0, -25.0, 0.0, 0.0, 0.0])
            success = self.pilot.frame_force_mode(
                wrench=release_ft,
                compliant_axes=[1, 1, 1, 0, 0, 0],
                distance=0.04,
                time_out=5.0,
                frame='flange'
            )
            if not success:
                raise RuntimeError(
                    f"Error while trying to release the lock. Robot end-effector is probably still connected.")

        rospy.loginfo(f"Arm successfully released from the plug.")
        return self.uc.request_action(out.plug_released, out.stop)
