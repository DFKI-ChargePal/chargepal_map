""" This file implements the state >>ObservePlugScene<< """
from __future__ import annotations
# libs
import rospy
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
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Get user and configuration data
        job: Job = ud.job
        detector_fp = self.cfg.data['detector'][self.cfg.data[job.get_id()]['scene_detector']]
        if job.in_stop_mode() or job.in_recover_mode():
            raise StateMachineError(f"Job in an invalid mode. Interrupt process")
        rospy.loginfo('Start observing the plug scene')
        found, T_base2socket = self.pilot.find_target_pose(
            detector_fp=detector_fp, time_out=self.cfg.data['detector_time_out'])
        if found:
            if job.is_part_of_plug_in():
                job.interior_socket.T_base2socket_scene = sm.SE3(T_base2socket)
                job.interior_socket.T_base2socket_close_up = sm.SE3(T_base2socket)
            elif job.is_part_of_plug_out():
                job.exterior_socket.T_base2socket_scene = sm.SE3(T_base2socket)
            else:
                raise StateMachineError(f"Not treated job: {job}")
            job.enable_progress_mode()
            outcome = out.plug_scene_obs
        else:
            if job.retry_count < 4:
                job.enable_retry_mode()
                outcome = out.err_obs_plug_retry
            else:
                job.enable_recover_mode()
                outcome = out.err_obs_plug_recover
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
