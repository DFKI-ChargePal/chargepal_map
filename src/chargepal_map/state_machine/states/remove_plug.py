""" This file implements the state >>RemovePlug<< """

from __future__ import annotations
# libs
import rospy
from smach import State

from chargepal_map.core import job_ids
from chargepal_map.ui.user_client import UserClient
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any
from ur_pilot import Pilot


class RemovePlug(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.stop, out.plug_removed_do, out.plug_removed_no], 
                       input_keys=['job_id'],
                       output_keys=['job_id'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start removing the plug from the socket')
        # Check for job ID
        job_id = ud.job_id
        # Get next output
        if job_id in job_ids.plug_in():
            new_out = out.plug_removed_do
        elif job_id in job_ids.plug_out():
            new_out = out.plug_removed_no
        else:
            raise ValueError(f"Invalid or undefined job ID '{job_id}' for this state.")
        # Get plug type
        if job_id in job_ids.type2_female():
            plug_type = 'type2_female'
        elif job_id in job_ids.type2_male():
            plug_type = 'type2_male'
        elif job_id in job_ids.ccs_female():
            plug_type = 'ccs_female'
        else:
            raise ValueError(f"Invalid or undefined job ID '{job_id}' for this state.")
        # Start removing procedure
        with self.pilot.plug_model.context(plug_type):
            sus_rm_plug, lin_ang_err = self.pilot.try2_remove_plug()
            rospy.loginfo(f"Removing plug from socket successfully: {sus_rm_plug}")
            rospy.logdebug(f"Final error after removing plug from socket: "
                           f"(Linear error={lin_ang_err[0]}[m] | Angular error={lin_ang_err[1]}[rad])")
        if not sus_rm_plug:
            raise RuntimeError(f"Error while trying to unplug. Plug is probably still connected.")
        return self.uc.request_action(new_out, out.stop)
