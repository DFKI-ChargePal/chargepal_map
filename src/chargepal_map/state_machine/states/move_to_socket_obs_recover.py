""" This file implements the state >>MoveToSocketObsRecover<< """
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
    StateMachineError,
)

# typing
from typing import Any
from ur_pilot import Pilot


class MoveToSocketObsRecover(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.socket_pre_obs, out.job_stopped], 
                       input_keys=['job'], 
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Get user and configuration data
        job: Job = ud.job
        job_data = self.cfg.data.get(job.ID)
        if not job.in_recover_mode():
            raise StateMachineError(f"Job {job} in an invalid mode. Interrupt process")

        rospy.loginfo(f"Moving back to the starting socket")
        with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(wps=job_data['joint_waypoints'], vel=job_data['vel'], acc=job_data['acc'])

        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.socket_pre_obs, out.job_stopped)
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
