""" This file implements the state >>FlipArm<< """
from __future__ import annotations

# libs
import rospy
import ur_pilot
import numpy as np
from smach import State

from chargepal_map.core import job_ids
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any
from ur_pilot import Pilot


class FlipArm(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.stop, out.arm_ready_to_plug],
                       input_keys=['job_id'],
                       output_keys=['job_id'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo(f"Start flipping the arm into the other workspace.")
        job_id = ud.job_id
        if job_id in job_ids.ccs_female():
            wps = self.cfg.data['wps_flip_to_rs']
        elif job_id in job_ids.type2_female() + job_ids.type2_male():
            wps = self.cfg.data['wps_flip_to_ls']
        else:
            raise RuntimeError(f"Can't match job id '{job_id}' to state action.")
        start_pos = self.pilot.robot.joint_pos
        first_pos = np.array(wps[1])
        error_pos = np.abs(first_pos - start_pos)
        if np.all(error_pos > self.cfg.data['max_moving_tolerance']):
            raise RuntimeError(f"Distance to first way points is to large: {error_pos}. To dangerous to move ;)")
        with self.pilot.context.position_control():
            self.pilot.robot.move_path_j(wps)
        rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
        outcome = out.arm_ready_to_plug
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(outcome, out.stop)
        return outcome
