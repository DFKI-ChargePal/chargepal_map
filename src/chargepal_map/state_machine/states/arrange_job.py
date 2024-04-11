""" This file implements the state >>Arrange Job<< """
from __future__ import annotations
# libs
import rospy
import numpy as np
from smach import State

from chargepal_map.core import job_ids
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig

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
                                 out.arm_ready_to_plug,
                                 out.arm_ready_to_free,
                                 out.arm_ready_to_move_ls,
                                 out.arm_ready_to_move_rs,
                                 out.stop],
                       input_keys=['job_id'],
                       output_keys=['job_id'])

    def execute(self, ud) -> str:
        print(), rospy.loginfo(f"Arrange the start of the state machine with respect to the job ID")
        job_id = ud.job_id
        # Check job id
        if not job_ids.is_valid(job_id):
            raise ValueError(f"Invalid job ID: {job_id}")
        # Connect to robot arm
        self.pilot.connect()
        # Get current workspace
        shoulder_pan_pos = self.pilot.robot.joint_pos[0]
        shoulder_pan_ws_ls, shoulder_pan_ws_rs = self.cfg.data['sd_pan_ws_ls'], self.cfg.data['sd_pan_ws_rs']
        is_ws_ls = True if shoulder_pan_ws_ls - np.pi/2 < shoulder_pan_pos < shoulder_pan_ws_ls + np.pi/2 else False
        is_ws_rs = True if shoulder_pan_ws_rs - np.pi/2 < shoulder_pan_pos < shoulder_pan_ws_rs + np.pi/2 else False
        
        if job_id == job_ids.free_drive_arm:
            outcome = out.arm_ready_to_free
        elif job_id == job_ids.move_home_arm:
            if is_ws_ls:
                outcome = out.arm_ready_to_move_ls
            elif is_ws_rs:
                outcome = out.arm_ready_to_move_rs
            else:
                raise RuntimeError(f"Robot arm is in an undefined workspace. Shoulder pan position: {shoulder_pan_pos}")
        elif job_id in job_ids.plug_in() + job_ids.plug_out():
            # Get desired workspace
            if job_id in job_ids.workspace_left():
                target_ws = 'ws_ls'
            elif job_id in job_ids.workspace_right():
                target_ws = 'ws_rs'
            else:
                raise ValueError(f"Not treated job ID: {job_id}")
            # Check if the robot arm has to be flipped
            if (is_ws_ls and target_ws == 'ws_rs') or (is_ws_rs and target_ws == 'ws_ls'):
                outcome = out.arm_in_wrong_ws
            else:
                outcome = out.arm_ready_to_plug
        else:
            raise ValueError(f"Not treated job ID: {job_id}")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(outcome, out.stop)
        return outcome
