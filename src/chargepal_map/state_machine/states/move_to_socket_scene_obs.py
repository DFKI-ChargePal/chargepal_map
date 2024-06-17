""" This file implements the state >>MoveToSocketSceneObs<< """
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


class MoveToSocketSceneObs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.socket_scene_pre_obs, out.job_stopped],
                       input_keys=['job'],
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Get user and configuration data
        job: Job = ud.job
        job_data = self.cfg.data.get(job.get_id())
        vel = self.cfg.data['vel']
        acc = self.cfg.data['acc']
        if job_data is None:
            raise KeyError(f"Can't find configuration data for the job: {job}")
        if job.in_progress_mode():
            rospy.loginfo(f"Start moving the arm to the job scene according its waypoints")
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(wps=job_data['joint_waypoints'], vel=vel, acc=acc)
        elif job.in_retry_mode():
            rospy.loginfo(f"Start moving the arm in an observation pose again with slightly different view angle.")
            with self.pilot.context.position_control():
                j_pos_finale = job_data['joint_waypoints'][-1]
                self.pilot.robot.movej(j_pos_finale, vel=vel, acc=acc)
                self.pilot.set_tcp(ur_pilot.EndEffectorFrames.CAMERA)
                current_ee_pose = self.pilot.get_pose(ur_pilot.EndEffectorFrames.CAMERA)
                theta = 5.0
                if job.retry_count % 4 == 1:
                    new_ee_pose = current_ee_pose * sm.SE3().Rx(theta, unit='deg')
                elif job.retry_count % 4 == 2:
                    new_ee_pose = current_ee_pose * sm.SE3().Ry(theta, unit='deg')
                elif job.retry_count % 4 == 3:
                    new_ee_pose = current_ee_pose * sm.SE3().Rx(-theta, unit='deg')
                else:  # job.retry_count % 4 == 0:
                    new_ee_pose = current_ee_pose * sm.SE3().Ry(-theta, unit='deg')
                self.pilot.move_to_tcp_pose(new_ee_pose)
            self.pilot.set_tcp(ur_pilot.EndEffectorFrames.FLANGE)
        else:
            raise StateMachineError(f"Job in an invalid mode. Interrupt process")
        outcome = out.socket_scene_pre_obs
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
