""" This file implements the state >>Completion<< """
from __future__ import annotations

# libs
import rospy
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


class Completion(State):

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
        job: Job = ud.job
        # cfg_data = self.cfg.extract_data("")
        rospy.loginfo(f"Complete process successfully in its finale state.")
        job.enable_stop_mode()
        job.track_state(type(self))
        print(state_footer(type(self)))
        return out.job_complete


class Incompletion(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self,
                       outcomes=[out.job_incomplete],
                       input_keys=['job' 'cart_name', 'station_name'],
                       output_keys=['job' 'cart_name', 'station_name'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        job: Job = ud.job
        # cfg_data = self.cfg.extract_data("")
        rospy.logwarn(f"Complete process unsuccessfully! However, the robot ended in a safe state.")
        job.enable_stop_mode()
        job.track_state(type(self))
        print(state_footer(type(self)))
        return out.job_incomplete
