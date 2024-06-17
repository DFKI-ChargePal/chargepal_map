""" This file implements the state >>ObservePlug<< """
from __future__ import annotations

# libs
import rospy
import ur_pilot
import numpy as np
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


class ObservePlug(State):

    _T_socket_save2camera = sm.SE3().Rt(R=sm.SO3.EulerVec((0.0, 0.0, -np.pi/2)), t=(-0.1, 0.0, -0.25)) * sm.SE3().Rx(-10.0, unit='deg')

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.plug_obs, out.err_obs_plug_recover, out.job_stopped], 
                       input_keys=['job'], 
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Get user and configuration data
        job: Job = ud.job
        if job.in_stop_mode() or job.in_recover_mode():
            raise StateMachineError(f"Job in an invalid mode. Interrupt process")
        found_plug = False
        if job.is_part_of_plug_in():
            T_base2socket_close_up = job.interior_socket.T_base2socket_close_up
            if T_base2socket_close_up is None:
                raise StateMachineError(f"Missing observation of the plug. Interrupt process")
            found_plug = True
        elif job.is_part_of_plug_out():
            T_base2socket_scene = job.exterior_socket.T_base2socket_scene
            if T_base2socket_scene is None:
                raise StateMachineError(f"Missing observation of the plug scene. Interrupt process")
            plug_dtt = self.cfg.data['detector'][self.cfg.data[job.get_plug_type()]['detector']]
            with self.pilot.plug_model.context(plug_type=job.get_plug_type()):
                with self.pilot.context.position_control():
                    self.pilot.set_tcp(ur_pilot.EndEffectorFrames.CAMERA)
                    T_base2camera = T_base2socket_scene * self._T_socket_save2camera
                    self.pilot.robot.movel(T_base2camera, self.cfg.data['vel'], self.cfg.data['acc'])
                found_plug, T_base2socket_close_up = self.pilot.find_target_pose(
                    detector_fp=plug_dtt, time_out=self.cfg.data['detector_time_out'])
            if found_plug:
                job.exterior_socket.T_base2socket_close_up = sm.SE3(T_base2socket_close_up)
        else:
            raise StateMachineError(f"Invalid or undefined job '{job}' for this state")
        if found_plug:
            rospy.loginfo(f"Found a plug pose.")
            rospy.logdebug(f"Transformation: Base-Socket = {ur_pilot.utils.se3_to_str(T_base2socket_close_up)}")
            outcome = out.plug_obs
        else:
            rospy.loginfo(f"Didn't find a proper plug pose.")
            job.enable_recover_mode()
            outcome = out.err_obs_plug_recover
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
