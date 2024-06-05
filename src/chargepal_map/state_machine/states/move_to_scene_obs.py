""" This file implements the state >>MoveToSceneObs<< """
from __future__ import annotations
# libs
import rospy
import ur_pilot
import numpy as np
from smach import State
import spatialmath as sm

from chargepal_map.core import job_ids
from chargepal_map.state_machine.outcomes import out
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.utils import StateMachineError

# typing
from typing import Any
from ur_pilot import Pilot


class MoveToSceneObs(State):

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
        # if job_id == job_ids.plug_out_ads_ac:
        #     rospy.loginfo(f"Start moving the arm to the adapter station on the left side")
        #     with self.pilot.context.position_control():
        #         self.pilot.robot.move_path_j(self.cfg.data[job_id]['joint_waypoints'],
        #                                      self.cfg.data['vel'],
        #                                      self.cfg.data['acc'])
        # elif job_id == job_ids.plug_out_ads_dc:
        #     rospy.loginfo(f"Start moving the arm to the adapter station on the right side")
        #     with self.pilot.context.position_control():
        #         self.pilot.robot.move_path_j(self.cfg.data[job_id]['joint_waypoints'],
        #                                      self.cfg.data['vel'],
        #                                      self.cfg.data['acc'])
        # elif job_id == job_ids.plug_out_bcs_ac:
        #     rospy.loginfo(f"Start moving the arm to the battery charging station on the left side")
        #     with self.pilot.context.position_control():
        #         self.pilot.robot.move_path_j(self.cfg.data[job_id]['joint_waypoints'],
        #                                      self.cfg.data['vel'],
        #                                      self.cfg.data['acc'])
        # else:
        #     raise StateMachineError(f"Invalid or undefined job ID '{job_id}' for this state.")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.plug_pre_obs, out.stop)
        return outcome
