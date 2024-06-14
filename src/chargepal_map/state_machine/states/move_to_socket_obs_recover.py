""" This file implements the state >>MoveToSocketObsRecover<< """
from __future__ import annotations

# libs
import rospy
from smach import State

from chargepal_map.job import Job
from chargepal_map.state_machine import outcomes as out
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.states.observe_socket import ObserveSocket
from chargepal_map.state_machine.states.observe_socket_scene import ObserveSocketScene
from chargepal_map.state_machine.utils import (
    state_name,
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
        job_key = job.get_id()
        if not job.in_recover_mode():
            raise StateMachineError(f"Job {job} in an invalid mode. Interrupt process")

        # Find matching key for motion path configuration
        if job.latest_state() == state_name(ObserveSocket):
            state_key = 'observe_socket'
        elif job.latest_state() == state_name(ObserveSocketScene):
            state_key = 'observe_socket_scene'
        else:
            raise StateMachineError(f"Latest state '{job.latest_state}' cannot be matched to a new state outcome")
        
        act_values = self.cfg.data[state_key][job_key]
        rospy.loginfo(f"Moving back to the starting socket")
        with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(wps=act_values, vel=self.cfg.data['vel'], acc=self.cfg.data['acc'])
        outcome = out.socket_pre_obs
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
