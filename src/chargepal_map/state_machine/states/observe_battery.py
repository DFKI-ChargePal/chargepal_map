""" This file implements the state >>ObserveBattery<< """
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
from chargepal_map.state_machine.utils import (
    state_header,
    state_footer,
    StateMachineError,
)

# typing
from typing import Any
from ur_pilot import Pilot


class ObserveBattery(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[
                           out.battery_obs_plug_in,
                           out.battery_obs_plug_out,
                           out.err_obs_recover,
                           out.job_stopped], 
                       input_keys=['job', 'battery_id'],
                       output_keys=['job', 'battery_id'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        job: Job = ud.job
        cfg_data = self.cfg.extract_data(ud.battery_id)
        found_socket, outcome = False, ''
        if self.user_cb is not None:
            rospy.loginfo(f"Ready to observe battery")
            outcome = self.user_cb.request_action(outcome, out.job_stopped)

        if outcome != out.job_stopped:
            socket_dtt = self.cfg.detector_files[cfg_data[job.get_plug_type()]['detector']]
            found_socket, T_base2socket_close_up = self.pilot.find_target_pose(
                detector_fp=socket_dtt, time_out=cfg_data['detector_time_out'])
            if found_socket:
                rospy.loginfo(f"Found a socket pose.")
                rospy.logdebug(f"Transformation: Base-Socket = {ur_pilot.utils.se3_to_str(T_base2socket_close_up)}")
                job.interior_socket.T_base2socket_close_up = sm.SE3(T_base2socket_close_up)
                if job.is_part_of_plug_in():
                    outcome = out.battery_obs_plug_in
                elif job.is_part_of_plug_out():
                    outcome = out.battery_obs_plug_out
                else:
                    raise StateMachineError(f"Invalid or undefined job '{job}' for this state")
            else:
                job.enable_recover_mode()
                outcome = out.err_obs_recover
                rospy.loginfo(f"Didn't find a proper socket pose")
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
