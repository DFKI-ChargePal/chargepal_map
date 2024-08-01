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
        # if self.user_cb is not None:
        #     rospy.loginfo(f"Ready to observe socket actively")
        #     outcome = self.user_cb.request_action(outcome, out.job_stopped)
        # if outcome != out.job_stopped:
        #     if job.in_progress_mode() or job.in_retry_mode():
        #         rospy.loginfo('Start observing the socket from a close-up view')
        #         if job.is_part_of_plug_out():
        #             T_base2socket_close_up = job.interior_socket.T_base2socket_close_up
        #             if T_base2socket_close_up is None:
        #                 raise StateMachineError(f"Missing observation of the socket. Interrupt process")
        #             found_socket = True
        #         elif job.is_part_of_plug_in():
        #             T_base2socket_scene = job.exterior_socket.T_base2socket_scene
        #             if T_base2socket_scene is None:
        #                 raise StateMachineError(f"Missing observation of the plug scene. Interrupt process")
        #             socket_dtt = self.cfg.data['detector'][self.cfg.data[job.get_plug_type()]['detector']]
        #             found_socket, T_base2socket_close_up = self.pilot.find_target_pose(
        #                 detector_fp=socket_dtt, time_out=self.cfg.data['detector_time_out'])
        #             if found_socket:
        #                 job.exterior_socket.T_base2socket_close_up = sm.SE3(T_base2socket_close_up)
        #         else:
        #             raise StateMachineError(f"Invalid or undefined job '{job}' for this state")
        #     elif job.in_recover_mode():
        #         rospy.loginfo('Skip observation process. Should be already done before')
        #         if job.is_part_of_plug_in():
        #             T_base2socket_close_up = job.interior_socket.T_base2socket_close_up
        #         elif job.is_part_of_plug_out():
        #             T_base2socket_close_up = job.exterior_socket.T_base2socket_close_up
        #         else:
        #             raise StateMachineError(f"Invalid or undefined job '{job}' for this state")
        #         if T_base2socket_close_up is None:
        #             raise StateMachineError(f"Missing observation of the socket. Interrupt process")
        #         found_socket = True
        #     else:  # job.in_stop_mode():
        #         raise StateMachineError(f"Job {job} in an invalid mode. Interrupt process")
        #     if found_socket:
        #         rospy.loginfo(f"Found a socket pose.")
        #         rospy.logdebug(f"Transformation: Base-Socket = {ur_pilot.utils.se3_to_str(T_base2socket_close_up)}")
        #         outcome = out.socket_obs
        #     else:
        #         rospy.loginfo(f"Didn't find a proper socket pose")
        #         job.enable_recover_mode()
        #         outcome = out.err_obs_socket_recover
        outcome = out.periphery_plug_obs + out.periphery_socket_obs + out.err_obs_retry + out.err_plug_in_recover + out.err_plug_out_recover
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
