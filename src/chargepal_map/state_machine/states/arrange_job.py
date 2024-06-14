""" This file implements the state >>Arrange Job<< """
from __future__ import annotations

# libs
import rospy
import numpy as np
from smach import State

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


class ArrangeJob(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self,
                       outcomes=[out.arm_in_wrong_ws,
                                 out.arm_ready_to_go,
                                 out.arm_ready_to_free,
                                 out.arm_ready_to_move_ls,
                                 out.arm_ready_to_move_rs,
                                 out.job_stopped],
                       input_keys=['job'],
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        rospy.loginfo(f"Arrange the start of the state machine with respect to the job ID")
        job: Job = ud.job
        # Get current workspace
        shoulder_pan_pos = self.pilot.robot.joint_pos[0]
        shoulder_pan_ws_ls= self.cfg.data['workspace_left']['shoulder_pan_pos']
        shoulder_pan_ws_rs = self.cfg.data['workspace_right']['shoulder_pan_pos']
        is_ws_ls = True if shoulder_pan_ws_ls - np.pi/2 < shoulder_pan_pos < shoulder_pan_ws_ls + np.pi/2 else False
        is_ws_rs = True if shoulder_pan_ws_rs - np.pi/2 < shoulder_pan_pos < shoulder_pan_ws_rs + np.pi/2 else False
        rospy.logdebug(f"Arm in workspace left: {is_ws_ls}")
        rospy.logdebug(f"Arm in workspace right: {is_ws_rs}")
        if job.get_id() == Job.ID.free_drive_arm:
            outcome = out.arm_ready_to_free
        elif job.get_id() == Job.ID.move_home_arm:
            if is_ws_ls:
                outcome = out.arm_ready_to_move_ls
            elif is_ws_rs:
                outcome = out.arm_ready_to_move_rs
            else:
                raise StateMachineError(f"Robot arm is in an undefined workspace. Shoulder pan position: {shoulder_pan_pos}")
        elif job.is_part_of_plug_in() or job.is_part_of_plug_out():
            # Get desired target joint positions
            if is_ws_ls:
                des_joint_pos = np.array(self.cfg.data['workspace_left']['home_joint_pos'])
            elif is_ws_rs:
                des_joint_pos =  np.array(self.cfg.data['workspace_left']['home_joint_pos'])
            else:
                raise StateMachineError(f"Robot arm is in an undefined workspace. Shoulder pan position: {shoulder_pan_pos}")
            # Get desired workspace
            if job.is_part_of_workspace_left():
                target_ws = 'ws_ls'
            elif job.is_part_of_workspace_right():
                target_ws = 'ws_rs'
            else:
                raise StateMachineError(f"Not treated job: {job}")
            # Check if robot is close to its home pos
            act_joint_pos = self.pilot.robot.joint_pos
            error_pos = np.abs(des_joint_pos - act_joint_pos)
            if np.all(error_pos > 1e-2):
                rospy.logwarn(f"Robot not in home position. This fact does not allow to start plugging.")
                outcome = out.job_stopped
            # Check if the robot arm has to be flipped
            elif (is_ws_ls and target_ws == 'ws_rs') or (is_ws_rs and target_ws == 'ws_ls'):
                outcome = out.arm_in_wrong_ws
            else:
                outcome = out.arm_ready_to_go
        else:
            raise StateMachineError(f"Not treated job: {job}")
        job.enable_progress_mode()
        job.track_state(type(self))
        rospy.loginfo(f"Chosen state machine job is: {job}")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        rospy.logdebug(f"Continue with outcome: {outcome}")
        print(state_footer(type(self)))
        return outcome
