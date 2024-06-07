""" This file implements the state >>MoveToSocketSceneObs<< """
from __future__ import annotations
# libs
import rospy
import ur_pilot
import numpy as np
from smach import State
import spatialmath as sm

from chargepal_map.job import Job
from chargepal_map.state_machine import outcomes as out
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.utils import StateMachineError

# typing
from typing import Any
from ur_pilot import Pilot


class MoveToSocketSceneObs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.socket_scene_pre_obs, out.job_stopped],
                       input_keys=['job'],
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print()
        job: Job = ud.job
        job_data = self.cfg.data.get(job)

        if job_data is None:
            raise KeyError(f"Can't find configuration data for the job: {job}")
        rospy.loginfo(f"Start moving the arm to the job scene")


        if job.is_part_of_plug_in():
            if job.is_part_of_workspace_left():
                rospy.loginfo(f"Start moving the arm to the battery station scene in the left workspace side") 
                # TODO: Add functionality
            elif job.is_part_of_workspace_right():
                rospy.loginfo(f"Start moving the arm to the battery station scene in the right workspace side") 
                # TODO: Add functionality
            else:
                raise StateMachineError(f"Invalid or undefined job ID '{job}' for this state.")
        # elif job.is_part_of_
        # if job == job_ids.plug_out_ads_ac:
        #     rospy.loginfo(f"Start moving the arm to the adapter station on the left side")
        #     with self.pilot.context.position_control():
        #         self.pilot.robot.move_path_j(self.cfg.data[job]['joint_waypoints'],
        #                                      self.cfg.data['vel'],
        #                                      self.cfg.data['acc'])
        # elif job == job_ids.plug_out_ads_dc:
        #     rospy.loginfo(f"Start moving the arm to the adapter station on the right side")
        #     with self.pilot.context.position_control():
        #         self.pilot.robot.move_path_j(self.cfg.data[job]['joint_waypoints'],
        #                                      self.cfg.data['vel'],
        #                                      self.cfg.data['acc'])
        # elif job == job_ids.plug_out_bcs_ac:
        #     rospy.loginfo(f"Start moving the arm to the battery charging station on the left side")
        #     with self.pilot.context.position_control():
        #         self.pilot.robot.move_path_j(self.cfg.data[job]['joint_waypoints'],
        #                                      self.cfg.data['vel'],
        #                                      self.cfg.data['acc'])
        # else:
        #     raise StateMachineError(f"Invalid or undefined job ID '{job}' for this state.")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.socket_scene_pre_obs, out.job_stopped)
        return outcome
