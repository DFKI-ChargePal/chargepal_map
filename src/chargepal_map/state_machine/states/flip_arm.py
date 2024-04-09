""" This file implements the state >>FlipArm<< """
from __future__ import annotations

# libs
import rospy
import ur_pilot
import numpy as np
from smach import State

from chargepal_map.core import job_ids
from chargepal_map.ui.user_client import UserClient
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any
from ur_pilot import Pilot


class FlipArm(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.stop, out.arm_ready_do, out.arm_ready_no],
                       input_keys=['job_id'],
                       output_keys=['job_id'])

    def execute(self, ud: Any) -> str:
        print()
        job_id = ud.job_id
        # Get current workspace
        shoulder_pan_pos = self.pilot.robot.joint_pos[0]
        shoulder_pan_ws_ls, shoulder_pan_ws_rs = self.cfg.data.sd_pan_ws_ls, self.cfg.data.sd_pan_ws_rs
        is_ws_ls = True if shoulder_pan_ws_ls - np.pi/2 < shoulder_pan_pos < shoulder_pan_ws_ls + np.pi/2 else False
        is_ws_rs = True if shoulder_pan_ws_rs - np.pi/2 < shoulder_pan_pos < shoulder_pan_ws_rs + np.pi/2 else False

        if is_ws_ls and is_ws_rs:
            raise RuntimeError(f"Can not determine the arm workspace state. "
                               f"Workspace intersection for shoulder pan position {shoulder_pan_pos}!")
        elif not is_ws_ls and not is_ws_rs:
            raise RuntimeError(f"Can not determine the arm workspace state. "
                               f"Shoulder pan position {shoulder_pan_pos} is outside of the workspace!")
        elif is_ws_ls:
            rospy.loginfo('Start flipping the arm from workspace on the left side to workspace on the right side')
            wps = self.cfg.data.flip_to_rs_wps
        elif is_ws_rs:
            rospy.loginfo('Start flipping the arm from workspace on the right side to workspace on the left side')
            wps = self.cfg.data.flip_to_ls_wps
        else:
            raise RuntimeError(f"Process is in an undefined situation. You should stop the process!")
        self.pilot.robot.move_path_j(wps)
        rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
        if job_id in job_ids.plug_in():
            outcome = out.arm_ready_no
        elif job_id in job_ids.plug_out():
            outcome = out.arm_ready_do
        else:
            raise ValueError(f"Invalid or unregistered job ID: {job_id}")
        return self.uc.request_action(outcome, out.stop)
