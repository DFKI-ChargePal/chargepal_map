""" This file implements the state >>MoveToCompletion<< """
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
)

# typing
from typing import Any
from ur_pilot import Pilot


class MoveToCompletion(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.job_complete, out.job_stopped],
                       input_keys=['job'],
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        job: Job = ud.job
        cfg_data = self.cfg.extract_data(ud.battery_id)
        wps = cfg_data[job.get_id()]['joint_waypoints']
        outcome = ''
        if self.user_cb is not None:
            rospy.loginfo(f"Ready to move arm in final position")
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        if outcome != out.job_stopped:
            rospy.loginfo(f"Start moving the arm to a save driving position.")
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(wps, cfg_data['vel'], cfg_data['acc'])
            rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
            outcome = out.job_complete
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
