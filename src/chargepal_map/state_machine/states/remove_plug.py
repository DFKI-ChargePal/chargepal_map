""" This file implements the state >>RemovePlug<< """
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


class RemovePlug(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[
                           out.battery_plug_removed,
                           out.periphery_plug_removed,
                           out.err_plug_out_stop,
                           out.job_stopped],
                       input_keys=['job'],
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Get user and configuration data
        job: Job = ud.job
        cfg_data = self.cfg.extract_data(ud.battery_id)
        if not job.in_progress_mode():
            raise StateMachineError(f"Job in an invalid mode. Interrupt process")
        outcome = ''
        if self.user_cb is not None:
            rospy.loginfo(f"Ready to remove plug from socket")
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        if outcome != out.job_stopped:
            rospy.loginfo('Start to try removing the plug from the socket')
            with self.pilot.plug_model.context(plug_type=job.get_plug_type()):
                with self.pilot.context.force_control():
                    sus_rm_plug, lin_ang_err = self.pilot.try2_remove_plug(
                        time_out=cfg_data['remove_time_out'],
                        max_force=cfg_data['remove_max_force'],
                        remove_tolerance=cfg_data['remove_tolerance']
                        )
                    rospy.loginfo(f"Removing plug from socket successfully: {sus_rm_plug}")
                    rospy.logdebug(f"Final error after removing plug from socket: "
                                f"(Linear error={lin_ang_err[0]}[m] | Angular error={lin_ang_err[1]}[rad])")
            if not sus_rm_plug:
                job.enable_stop_mode()
                outcome = out.err_plug_out_stop
                rospy.logerr(f"Robot was not able to remove the plug successfully. Plug is probably still connected")
            else:
                rospy.loginfo(f"Robot removed the plug successfully")
                if job.is_part_of_plug_in():
                    outcome = out.battery_plug_removed
                elif job.is_part_of_plug_out():
                    outcome = out.periphery_plug_removed
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
