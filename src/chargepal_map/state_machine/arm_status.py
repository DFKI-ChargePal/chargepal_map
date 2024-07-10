from __future__ import annotations
# libs
import rospy
import numpy as np
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.states.arrange_job import ArrangeJob

# services
from chargepal_services.srv import getArmStatus, getArmStatusRequest, getArmStatusResponse

# typing
from typing import Any
from ur_pilot import Pilot


class ArmStatusServer:

    def __init__(self, config: dict[str, Any]) -> None:
        self.arm_free = False
        self.cfg = StateConfig(type(ArrangeJob), config=config)
        self._srv = rospy.Service('get_arm_status', getArmStatus)

    def initial_check(self, pilot: Pilot) -> None:
        # Get current workspace
        shoulder_pan_pos = pilot.robot.joint_pos[0]
        shoulder_pan_ws_ls= self.cfg.data['workspace_left']['shoulder_pan_pos']
        shoulder_pan_ws_rs = self.cfg.data['workspace_right']['shoulder_pan_pos']
        is_ws_ls = True if shoulder_pan_ws_ls - np.pi/2 < shoulder_pan_pos < shoulder_pan_ws_ls + np.pi/2 else False
        is_ws_rs = True if shoulder_pan_ws_rs - np.pi/2 < shoulder_pan_pos < shoulder_pan_ws_rs + np.pi/2 else False
        # Get home position
        if is_ws_ls:
            set_joint_pos = np.array(self.cfg.data['workspace_left']['home_joint_pos'])
        elif is_ws_rs:
            set_joint_pos = np.array(self.cfg.data['workspace_right']['home_joint_pos'])
        # Check if robot is close to its home pos
        if is_ws_ls or is_ws_rs:
            cur_joint_pos = pilot.robot.joint_pos
            error_pos = np.abs(set_joint_pos - cur_joint_pos)
            if np.all(error_pos < 1e-2):
                self.arm_free = True
            else:
                self.arm_free = False
        else:
            self.arm_free = False

    def _status_callback(self, req: getArmStatusRequest) -> getArmStatusResponse:
        res = getArmStatusResponse()
        res.arm_free = self.arm_free
        return res
