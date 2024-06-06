""" This file implements the state >>RemovePlug<< """

from __future__ import annotations
# libs
import rospy
from smach import State

from chargepal_map.job import Job
from chargepal_map.state_machine import outcomes as out
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.utils import StateMachineError

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
                           out.plug_removed,
                           out.err_plug_out_stop,
                           out.job_stopped],
                       input_keys=['job'],
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start removing the plug from the socket')
        # Check for job ID
        job: Job = ud.job
        # Get next output
        if job in job_ids.plug_in():
            outcome = out.plug_removed_do
        elif job in job_ids.plug_out():
            outcome = out.plug_removed_no
        else:
            raise StateMachineError(f"Invalid or undefined job ID '{job}' for this state.")
        # Get plug type
        plug_type = job.get_plug_type()
        # Start removing procedure
        with self.pilot.plug_model.context(plug_type):
            with self.pilot.context.force_control():
                sus_rm_plug, lin_ang_err = self.pilot.try2_remove_plug(
                    time_out=self.cfg.data['remove_time_out'],
                    max_force=self.cfg.data['remove_max_force'],
                    remove_tolerance=self.cfg.data['remove_tolerance']
                    )
                rospy.loginfo(f"Removing plug from socket successfully: {sus_rm_plug}")
                rospy.logdebug(f"Final error after removing plug from socket: "
                               f"(Linear error={lin_ang_err[0]}[m] | Angular error={lin_ang_err[1]}[rad])")
        if not sus_rm_plug:
            raise StateMachineError(f"Error while trying to unplug. Plug is probably still connected.")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        return outcome
