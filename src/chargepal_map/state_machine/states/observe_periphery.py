""" This file implements the state >>ObservePeriphery<< """
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


class ObservePeriphery(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[
                           out.periphery_plug_obs,
                           out.periphery_socket_obs,
                           out.err_obs_retry,
                           out.err_plug_in_recover,
                           out.err_plug_out_recover,
                           out.job_stopped],
                       input_keys=['job'],
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        job: Job = ud.job
        found_socket, outcome = False, ''
        if self.user_cb is not None:
            rospy.loginfo(f"Ready to observe periphery")
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        if outcome != out.job_stopped:
            periphery_dtt = self.cfg.data['detector'][self.cfg.data[job.get_plug_type()]['detector']]
            found_socket, T_base2socket_scene = self.pilot.find_target_pose(
                detector_fp=periphery_dtt, time_out=self.cfg.data['detector_time_out'])
            if found_socket:
                rospy.loginfo(f"Found a pose of the socket scene.")
                rospy.logdebug(f"Transformation: Base-Socket = {ur_pilot.utils.se3_to_str(T_base2socket_scene)}")
                job.exterior_socket.T_base2socket_scene = sm.SE3(T_base2socket_scene)
                job.enable_progress_mode()
                if job.is_part_of_plug_in():
                    outcome = out.periphery_socket_obs
                elif job.is_part_of_plug_out():
                    outcome = out.periphery_plug_obs
                else:
                    raise StateMachineError(f"Invalid or undefined job '{job}' for this state")
            else:
                if job.retry_count < 4:
                    job.enable_retry_mode()
                    outcome = out.err_obs_retry
                    rospy.loginfo(f"No socket scene found. Try again for {4 - job.retry_count}")
                else:
                    job.enable_recover_mode()
                    if job.is_part_of_plug_in():
                        outcome = out.err_plug_in_recover
                    elif job.is_part_of_plug_out():
                        outcome = out.err_plug_out_recover
                    else:
                        raise StateMachineError(f"Invalid or undefined job '{job}' for this state")
                    rospy.loginfo(f"No socket scene found. Check observation view and the detector settings")
                    rospy.logwarn(f"Switch to recover mode")
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
