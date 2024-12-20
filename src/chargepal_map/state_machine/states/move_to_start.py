""" This file implements the states >>MoveToStartXx<< """
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


class MoveToStartLs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self,
                       outcomes=[out.job_complete],
                       input_keys=['job', 'cart_name', 'station_name'],
                       output_keys=['job', 'cart_name', 'station_name'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Check job id
        job: Job = ud.job
        cfg_data = self.cfg.extract_data(ud.cart_name)
        if job.get_id() != Job.ID.move_home_arm:
            raise StateMachineError(f"This state doesn't support the job: {job}")
        start_pos = self.pilot.robot.joint_pos
        home_pos = cfg_data['home_joint_pos']
        # Try to avoid dangerous movements
        error_pos = np.abs(np.array(home_pos) - start_pos)
        if np.all(error_pos > cfg_data['max_moving_tolerance']):
            raise StateMachineError(f"Distance to home position is to large: {error_pos}. To dangerous to move ;)")
        with self.pilot.context.position_control():
            self.pilot.robot.movej(home_pos, cfg_data['vel'], cfg_data['acc'])
        # Get finale position and check remaining error to target
        finale_pos = self.pilot.robot.joint_pos
        error_pos = np.abs(np.array(home_pos) - finale_pos)
        if np.all(error_pos > 1e-2):
            raise StateMachineError(f"Remaining distance to home position is to large: {error_pos}. "
                                    f"Is the robot running properly?")
        rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
        outcome = out.job_complete
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome


class MoveToStartRs(State): 

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.job_complete], 
                       input_keys=['job', 'cart_name', 'station_name'],
                       output_keys=['job', 'cart_name', 'station_name'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Check job id
        job: Job = ud.job
        cfg_data = self.cfg.extract_data(ud.cart_name)
        if job.get_id() != Job.ID.move_home_arm:
            raise StateMachineError(f"This state doesn't support the job: {job}")
        start_pos = self.pilot.robot.joint_pos
        home_pos = cfg_data['home_joint_pos']
        # Try to avoid dangerous movements
        error_pos = np.abs(np.array(home_pos) - start_pos)
        if np.all(error_pos > cfg_data['max_moving_tolerance']):
            raise StateMachineError(f"Distance to home position is to large: {error_pos}. To dangerous to move ;)")
        with self.pilot.context.position_control():
            self.pilot.robot.movej(home_pos, cfg_data['vel'], cfg_data['acc'])
        # Get finale position and check remaining error to target
        finale_pos = self.pilot.robot.joint_pos
        error_pos = np.abs(np.array(home_pos) - finale_pos)
        if np.all(error_pos > 1e-2):
            raise StateMachineError(f"Remaining distance to home position is to large: {error_pos}. "
                                    f"Is the robot is running properly?")
        rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
        outcome = out.job_complete
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
