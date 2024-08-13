""" This file implements the state >>ObservePlug<< """
from __future__ import annotations

# libs
import rospy
import ur_pilot
from smach import State
import spatialmath as sm

from chargepal_map.job import Job
from chargepal_map.state_machine import outcomes as out
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.utils import state_header, state_footer

# typing
from typing import Any
from ur_pilot import Pilot


class ObservePlug(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.plug_obs, out.err_obs_recover, out.job_stopped], 
                       input_keys=['job'], 
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Get user and configuration data
        job: Job = ud.job
        cfg_data = self.cfg.extract_data(ud.battery_id)
        outcome = ''
        if self.user_cb is not None:
            rospy.loginfo(f"Ready to observe the plug")
            outcome = self.user_cb.request_action(outcome, out.job_stopped)

        if outcome != out.job_stopped:
            plug_dtt = cfg_data['detector'][cfg_data[job.get_plug_type()]['detector']]
            found_plug, T_base2socket_close_up = self.pilot.find_target_pose(
                detector_fp=plug_dtt, time_out=cfg_data['detector_time_out'])
            if found_plug:
                rospy.loginfo(f"Found a plug pose.")
                rospy.logdebug(f"Transformation: Base-Socket = {ur_pilot.utils.se3_to_str(T_base2socket_close_up)}")
                job.exterior_socket.T_base2socket_close_up = sm.SE3(T_base2socket_close_up)
                outcome = out.plug_obs
            else:
                rospy.loginfo(f"Didn't find a proper plug pose.")
                job.enable_recover_mode()
                outcome = out.err_obs_recover
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
