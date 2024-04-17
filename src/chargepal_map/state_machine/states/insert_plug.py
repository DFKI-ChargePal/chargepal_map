""" This file implements the state >>InsertPlug<< """
from __future__ import annotations

# libs
import rospy
from smach import State
import spatialmath as sm

from chargepal_map.core import job_ids
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.utils import StateMachineError

# typing
from typing import Any
from ur_pilot import Pilot


class InsertPlug(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.stop, out.plug_connected],
                       input_keys=['job_id', 'T_base2socket'],
                       output_keys=['job_id', 'T_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start inserting the plug to the socket')
        job_id = ud.job_id
        # Get transformation matrices
        T_base2socket = sm.SE3(ud.T_base2socket)
        # Get plug type
        if job_id in job_ids.type2_female():
            plug_type = 'type2_female'
        elif job_id in job_ids.type2_male():
            plug_type = 'type2_male'
        elif job_id in job_ids.ccs_female():
            plug_type = 'ccs_female'
        else:
            raise StateMachineError(f"Invalid or undefined job ID '{job_id}' for this state.")
        # Start inserting procedure
        with self.pilot.plug_model.context(plug_type): 
            sus_eng_plug, sus_ins_plug = False, False
            with self.pilot.context.force_control():
                sus_eng_plug, lin_ang_err = self.pilot.try2_engage_with_socket(T_base2socket)
                rospy.logdebug(f"Final error after engaging between plug and socket: "
                               f"(Linear error={lin_ang_err[0]}[m] | Angular error={lin_ang_err[1]}[rad])")
                if sus_eng_plug:
                    sus_ins_plug, lin_ang_err = self.pilot.try2_insert_plug(T_base2socket)
                    rospy.logdebug(f"Final error after inserting plug to socket: "
                                   f"(Linear error={lin_ang_err[0]}[m] | Angular error={lin_ang_err[1]}[rad])")

            rospy.loginfo(f"Engaging plug to socket successfully: {sus_eng_plug}")
            rospy.loginfo(f"Inserting plug to socket successfully: {sus_ins_plug}")
        if not sus_eng_plug or not sus_ins_plug:
            raise StateMachineError(f"Spatial error to large. Robot is probably in an undefined condition.")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.plug_connected, out.stop)
        return outcome
