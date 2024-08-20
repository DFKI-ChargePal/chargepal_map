""" This file implements the state >>MoveToIncompletion<< """
from __future__ import annotations

# libs
import rospy
import ur_pilot
from smach import State

from chargepal_map.job import Job
from chargepal_map.state_machine import outcomes as out
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.states.attach_plug import AttachPlug
from chargepal_map.state_machine.states.observe_plug import ObservePlug
from chargepal_map.state_machine.states.release_plug import ReleasePlug
from chargepal_map.state_machine.states.observe_plug_id import ObservePlugId
from chargepal_map.state_machine.states.observe_battery import ObserveBattery
from chargepal_map.state_machine.states.observe_periphery import ObservePeriphery
from chargepal_map.state_machine.utils import (
    state_name,
    state_header,
    state_footer,
    StateMachineError,
)

# typing
from typing import Any
from ur_pilot import Pilot


class MoveToIncompletion(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.job_incomplete, out.job_stopped],
                       input_keys=['job', 'cart_name', 'station_name'],
                       output_keys=['job', 'cart_name', 'station_name'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        job: Job = ud.job
        cfg_data = self.cfg.extract_data(ud.cart_name)
        # Find matching key for motion path configuration
        plug_type_key = job.get_plug_type()
        if job.latest_state() == state_name(ObserveBattery):
            state_key = 'observe_battery'
            p2p_key = 'battery2home'
        elif job.latest_state() == state_name(ObservePeriphery):
            state_key = 'observe_periphery'
            p2p_key = 'periphery2home'
        elif job.latest_state() == state_name(ObservePlugId):
            state_key = 'observe_plug_id'
            if job.is_part_of_plug_in():
                p2p_key = 'battery2home'
            elif job.is_part_of_plug_out():
                p2p_key = 'periphery2home'
            else:
                raise StateMachineError(f"Current job '{job.ID}' cannot be matched to a waypoint set")
        elif job.latest_state() == state_name(ObservePlug):
            state_key = 'observe_plug'
            p2p_key = 'periphery2home'
        elif job.latest_state() == state_name(AttachPlug):
            state_key = 'attach_plug'
            if job.is_part_of_plug_in():
                p2p_key = 'battery2home'
            elif job.is_part_of_plug_out():
                p2p_key = 'periphery2home'
            else:
                raise StateMachineError(f"Current job '{job.ID}' cannot be matched to a waypoint set")
        elif job.latest_state() == state_name(ReleasePlug):
            state_key = 'release_plug'
            if job.is_part_of_plug_in():
                p2p_key = 'battery2home'
            elif job.is_part_of_plug_out():
                p2p_key = 'periphery2home'
            else:
                raise StateMachineError(f"Current job '{job.ID}' cannot be matched to a waypoint set")
        else:
            raise StateMachineError(f"Latest state '{job.latest_state()}' cannot be matched to a new state outcome")
        outcome = ''
        if self.user_cb is not None:
            rospy.loginfo(f"Ready to move arm in final position")
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        if outcome != out.job_stopped:
            rospy.loginfo(f"Start moving the arm to a save driving position.")
            act_values = cfg_data[plug_type_key][state_key][p2p_key]
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(act_values, cfg_data['vel'], cfg_data['acc'])
            rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
            outcome = out.job_incomplete
        job.enable_stop_mode()
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
