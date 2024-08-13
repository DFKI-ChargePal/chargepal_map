""" This file implements the state >>MoveToBatteryObs<< """
from __future__ import annotations
# libs
import rospy
import ur_pilot
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


class MoveToBatteryObs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.battery_pre_obs, out.job_stopped],
                       input_keys=['job'],
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Get user and configuration data
        job: Job = ud.job
        cfg_data = self.cfg.extract_data(ud.battery_id)
        vel = cfg_data['vel']
        acc = cfg_data['acc']
        outcome = out.battery_pre_obs
        # Find matching key for motion path configuration
        if job.is_part_of_workspace_right():
            job_key = 'workspace_right'
        elif job.is_part_of_workspace_left():
            job_key = 'workspace_left'
        else:
            raise StateMachineError(f"Current job '{job.ID}' cannot be matched to a new state outcome")
        if self.user_cb is not None:
            rospy.loginfo(f"Ready to move arm to the battery observation configuration")
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        if outcome != out.job_stopped:
            if job.in_progress_mode():
                rospy.loginfo(f"Start moving the arm to the battery observation")
                act_values = cfg_data[job_key]
                with self.pilot.context.position_control():
                    self.pilot.robot.movej(target=act_values, vel=vel, acc=acc)
                rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
            else:
                raise StateMachineError(f"Job in an invalid mode. Interrupt process")
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
