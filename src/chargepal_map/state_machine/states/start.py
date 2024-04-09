""" This file implements the state >>Start<< """
from __future__ import annotations
# libs
import rospy
import numpy as np
from smach import State

from chargepal_map.core import job_ids
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.step_by_user import StepByUserClient

# typing
from typing import Any
from ur_pilot import Pilot


class Start(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = StepByUserClient(self.cfg.data['step_by_user'])
        State.__init__(self,
                       outcomes=[out.stop, out.arm_in_wrong_ws, out.arm_ready_do, out.arm_ready_no],
                       input_keys=['job_id'],
                       output_keys=['job_id'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo(f"Check robot state against action conditions.")
        job_id = ud.job_id
        # Connect to robot arm
        self.pilot.connect()
        # Get current workspace
        shoulder_pan_pos = self.pilot.robot.joint_pos[0]
        shoulder_pan_ws_ls, shoulder_pan_ws_rs = self.cfg.data['sd_pan_ws_ls'], self.cfg.data['sd_pan_ws_rs']
        is_ws_ls = True if shoulder_pan_ws_ls - np.pi/2 < shoulder_pan_pos < shoulder_pan_ws_ls + np.pi/2 else False
        is_ws_rs = True if shoulder_pan_ws_rs - np.pi/2 < shoulder_pan_pos < shoulder_pan_ws_rs + np.pi/2 else False
        # Get desired workspace
        if job_id in job_ids.workspace_left():
            target_ws = 'ws_ls'
        elif job_id in job_ids.workspace_right():
            target_ws = 'ws_rs'
        else:
            raise ValueError(f"Invalid or unregistered job ID: {job_id}")
        # Get state outcome
        if (is_ws_ls and target_ws == 'ws_rs') or (is_ws_rs and target_ws == 'ws_ls'):
            outcome = out.arm_in_wrong_ws
        else:
            if job_id in job_ids.plug_in():
                outcome = out.arm_ready_no
            elif job_id in job_ids.plug_out():
                outcome = out.arm_ready_do
            else:
                raise ValueError(f"Invalid or unregistered job ID: {job_id}")
        rospy.loginfo(f"Start process with job id: {job_id}")
        return self.uc.request_action(outcome, out.stop)
