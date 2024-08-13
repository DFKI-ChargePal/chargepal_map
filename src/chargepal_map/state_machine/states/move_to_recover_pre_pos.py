""" This file implements the state >>MoveToRecoverPrePos<< """
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


class MoveToRecoverPrePos(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self,
                       outcomes=[out.recover_pre_pos, out.job_stopped],
                       input_keys=['job'],
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Try to find matching configuration
        job: Job = ud.job
        cfg_data = self.cfg.extract_data(ud.battery_id)
        vel = cfg_data['vel']
        acc = cfg_data['acc']
        if not job.in_recover_mode():
            raise StateMachineError(f"Job not in recover mode. Interrupt process")
        # Get infos and dictionary key for the right movement
        if job.is_part_of_plug_in():
            ws_key = 'move_to_battery'
            T_base2socket = job.interior_socket.T_base2socket_close_up
        elif job.is_part_of_plug_out():
            ws_key = 'move_to_periphery'
            T_base2socket = job.exterior_socket.T_base2socket_close_up
        else:
            raise StateMachineError(f"Invalid or undefined job '{job}' for this state.")
        plug_key = job.get_plug_type()
        path_values = cfg_data[ws_key][plug_key]['recover_waypoints']
        # Try to move back to the initial socket to recover
        outcome = out.recover_pre_pos
        if self.user_cb is not None:
            rospy.loginfo(f"Ready to move the arm back to the initial socket")
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        if outcome != out.job_stopped:
            rospy.loginfo(f"Start moving the arm to the initial socket scene")
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(path_values, vel, acc)
            rospy.loginfo(f"Start moving the arm to the pre-connect position in front of the socket")
            with self.pilot.plug_model.context(plug_type=job.get_plug_type()):
                with self.pilot.context.position_control():
                    sus, _ = self.pilot.try2_approach_to_socket(T_base2socket)
            rospy.loginfo(f"Arm ended in pre-insert pose for recovery successfully: {sus}")
            rospy.logdebug(f"Transformation: Base-TCP = {ur_pilot.utils.se3_to_str(self.pilot.robot.tcp_pose)}")
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
