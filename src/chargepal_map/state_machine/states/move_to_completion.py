""" This file implements the state >>MoveToCompletion<< """
from __future__ import annotations
# libs
import rospy
import ur_pilot
from smach import State

from chargepal_map.state_machine.outcomes import out
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any
from ur_pilot import Pilot


class MoveToCompletion(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.stop, out.completed],
                       input_keys=['job_id'],
                       output_keys=['job_id'])

    def execute(self, ud: Any) -> str:
        print()
        job_id = ud.job_id
        wps = self.cfg.data[job_id]['joint_waypoints']
        rospy.loginfo(f"Start moving the arm to a save driving position.")
        with self.pilot.context.position_control():
            self.pilot.robot.move_path_j(wps, self.cfg.data['vel'], self.cfg.data['acc'])
        rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
        outcome = out.completed
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.completed, out.stop)
        return outcome
