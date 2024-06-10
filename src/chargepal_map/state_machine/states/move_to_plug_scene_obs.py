""" This file implements the state >>MoveToPlugSceneObs<< """
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
from chargepal_map.state_machine.utils import StateMachineError

# typing
from typing import Any
from ur_pilot import Pilot


class MoveToPlugSceneObs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self,
                       outcomes=[out.plug_scene_pre_obs, out.job_stopped],
                       input_keys=['job'],
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print()
        job: Job = ud.job
        job_data = self.cfg.data.get(job.ID)

        if job_data is None:
            raise KeyError(f"Can't find configuration data for the job: {job}")
        if job.in_stop_mode() or job.in_recover_mode():
            raise StateMachineError(f"Job in an invalid mode. Interrupt process")
        # Move to battery box observation
        if job.is_part_of_plug_in():
            if job.in_progress_mode():
                # Start moving the arm
                rospy.loginfo(f"Start moving the arm to the battery box scene") 
                with self.pilot.context.position_control():
                        self.pilot.robot.move_j(job_data[job.ID]['joint_position'],
                                                job_data['vel'],
                                                job_data['acc'])
            elif job.in_retry_mode():
                rospy.loginfo(f"Retry action: "
                              f"Since there shouldn't be any variance in the observation, the robot will not move")
        elif job.is_part_of_plug_out():
            if job.in_progress_mode():
                rospy.loginfo(f"Start moving the arm to one of the outer side scene")
                with self.pilot.context.position_control():
                    self.pilot.robot.move_path_j(job_data[job.ID]['joint_waypoints'],
                                                 job_data['vel'],
                                                 job_data['acc'])
            elif job.in_retry_mode():
                # Move to a robot arm in a slightly different observation pose
                with self.pilot.context.position_control():
                    j_pos_finale = job_data[job.ID]['joint_waypoints'][-1]
                    self.pilot.robot.movej(j_pos_finale, job_data['vel'], job_data['acc'])
                    self.pilot.set_tcp(ur_pilot.EndEffectorFrames.CAMERA)
                    current_ee_pose = self.pilot.get_pose(ur_pilot.EndEffectorFrames.CAMERA)
                    if job.retry_count % 4 == 1:
                        new_ee_pose = current_ee_pose * sm.SE3().Rx(5, unit='deg')
                    elif job.retry_count % 4 == 2:
                        new_ee_pose = current_ee_pose * sm.SE3().Ry(5, unit='deg')
                    elif job.retry_count % 4 == 3:
                        new_ee_pose = current_ee_pose * sm.SE3().Rx(-5, unit='deg')
                    else:  # job.retry_count % 4 == 0:
                        new_ee_pose = current_ee_pose * sm.SE3().Ry(-5, unit='deg')
                    self.pilot.move_to_tcp_pose(new_ee_pose)
                self.pilot.set_tcp(ur_pilot.EndEffectorFrames.FLANGE)
        else:
            raise StateMachineError(f"Invalid or undefined job ID '{job}' for this state.")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.scene_pre_obs, out.job_stopped)
        return outcome
