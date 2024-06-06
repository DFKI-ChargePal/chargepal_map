""" This file implements the state >>MoveToSocketObs<< """
from __future__ import annotations
# libs
import rospy
from smach import State

from chargepal_map.job import Job
from chargepal_map.state_machine import outcomes as out
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.utils import StateMachineError

# typing
from typing import Any
from ur_pilot import Pilot


class MoveToSocketObs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.socket_pre_obs, out.job_stopped], 
                       input_keys=['job'], 
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print()
        job: Job = ud.job
        if job == job_ids.plug_in_ads_ac:
            rospy.loginfo(f"Start moving the plug to the adapter station on the left side")
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(self.cfg.data[job]['joint_waypoints'],
                                             self.cfg.data['vel'],
                                             self.cfg.data['acc'])
        elif job == job_ids.plug_in_ads_dc:
            rospy.loginfo(f"Start moving the plug to the adapter station on the right side")
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(self.cfg.data[job]['joint_waypoints'],
                                             self.cfg.data['vel'],
                                             self.cfg.data['acc'])
        elif job == job_ids.plug_in_bcs_ac:
            rospy.loginfo(f"Start moving the plug to the battery charging station on the left side")
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(self.cfg.data[job]['joint_waypoints'],
                                             self.cfg.data['vel'],
                                             self.cfg.data['acc'])
        else:
            raise StateMachineError(f"Invalid or undefined job ID '{job}' for this state.")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.socket_pre_obs, out.job_stopped)
        return outcome
