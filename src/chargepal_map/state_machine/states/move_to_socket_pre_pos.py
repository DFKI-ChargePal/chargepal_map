""" This file implements the state >>MoveToSocketPrePos<< """
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
    StateMachineError,
    state_header,
    state_footer,
)

# typing
from typing import Any
from ur_pilot import Pilot


class MoveToSocketPrePos(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, outcomes=[out.socket_pre_pos, out.job_stopped],
                       input_keys=['job'], 
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Get user and configuration data
        job: Job = ud.job
        if job.in_progress_mode() or job.in_retry_mode():
            exterior = job.is_part_of_plug_in()
            interior = job.is_part_of_plug_out()
        elif job.in_recovery_mode():
            exterior = job.is_part_of_plug_out()
            interior = job.is_part_of_plug_in()
        else:
            raise StateMachineError(f"Job in an invalid mode. Interrupt process")
        if exterior:
            T_base2socket = job.exterior_socket.T_base2socket_close_up
        elif interior:
            T_base2socket = job.interior_socket.T_base2socket_close_up
        else:
            raise StateMachineError(f"Invalid or undefined job '{job}' for this state")
        rospy.loginfo('Start moving the plug to the pre connecting pose')
        with self.pilot.plug_model.context(plug_type=job.get_plug_type()):
            with self.pilot.context.position_control():
                sus, _ = self.pilot.try2_approach_to_socket(T_base2socket)
        rospy.loginfo(f"Arm ended in pre-insert pose successfully: {sus}")
        rospy.logdebug(f"Transformation: Base-TCP = {ur_pilot.utils.se3_to_str(self.pilot.robot.tcp_pose)}")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.socket_pre_pos, out.job_stopped)
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
