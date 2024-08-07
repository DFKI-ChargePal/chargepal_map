""" This file implements the state >>FlipArm<< """
from __future__ import annotations

# libs
import rospy
import ur_pilot
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


class FlipArm(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.arm_ready_to_go, out.job_stopped],
                       input_keys=['job'],
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        job: Job = ud.job
        if not job.in_progress_mode():
            raise StateMachineError(f"Job is not in running mode. Interrupt process.")
        if job.is_part_of_ccs_female():
            wps = self.cfg.data['wps_flip_to_rs']
        elif job.is_part_of_type2_female() or job.is_part_of_type2_male():
            wps = self.cfg.data['wps_flip_to_ls']
        else:
            raise StateMachineError(f"Can't match job '{job}' to state action.")
        start_pos = self.pilot.robot.joint_pos
        first_pos = np.array(wps[1])
        error_pos = np.abs(first_pos - start_pos)
        if np.all(error_pos > self.cfg.data['max_moving_tolerance']):
            raise StateMachineError(f"Distance to first way points is to large: {error_pos}. To dangerous to move ;)")
        outcome = out.arm_ready_to_go
        if self.user_cb is not None:
            rospy.loginfo(f"Ready flipping the arm into the other workspace.")
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        if outcome != out.job_stopped:
            rospy.loginfo(f"Start flipping the arm into the other workspace.")
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(wps, vel=self.cfg.data['vel'], acc=self.cfg.data['acc'])
            rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
