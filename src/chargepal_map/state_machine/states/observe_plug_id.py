""" This file implements the state >>MoveToPlugObs<< """
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


class ObservePlugId(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[
                           out.battery_plug_id_obs,
                           out.periphery_plug_id_obs,
                           out.err_obs_recover,
                           out.job_stopped], 
                       input_keys=['job', 'battery_id'], 
                       output_keys=['job', 'battery_id'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Get user and configuration data
        job: Job = ud.job
        cfg_data = self.cfg.extract_data(ud.battery_id)
        outcome = ''
        if self.user_cb is not None:
            rospy.loginfo(f"Ready observe the plug ID")
            outcome = self.user_cb.request_action(outcome, out.job_stopped)

        if outcome != out.job_stopped:
            plug_id_dtt = self.cfg.detector_files[cfg_data[job.get_plug_type()]['detector']]
            found_plug_id, _ = self.pilot.find_target_pose(
                detector_fp=plug_id_dtt, time_out=cfg_data['detector_time_out'])
            if found_plug_id:
                rospy.loginfo(f"Found plug of type '{job.get_plug_type()}' in its intended place")
                if job.is_part_of_plug_in():
                    outcome = out.battery_plug_id_obs
                elif job.is_part_of_plug_out():
                    outcome = out.periphery_plug_id_obs
                else:
                    raise StateMachineError(f"Invalid or undefined job '{job}' for this state")
            else:
                job.enable_recover_mode()
                outcome = out.err_obs_recover
                rospy.loginfo(f"Can't find plug id marker. "
                              f"Plug of type '{job.get_plug_type()}' probably not in its place")
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
