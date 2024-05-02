""" This file implements the state >>ReleasePlug<< """
from __future__ import annotations
from calendar import c

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


class ReleasePlug(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.stop, out.plug_released], 
                       input_keys=['job_id', 'T_base2socket'],
                       output_keys=['job_id', 'T_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start releasing the arm from the plug')
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
        # Start releasing procedure
        with self.pilot.plug_model.context(plug_type):
            sus_unl_plug, sus_dec_plug = False, False
            with self.pilot.context.force_control():
                sus_unl_plug, lin_ang_err = self.pilot.try2_unlock_plug(
                    T_base2socket=T_base2socket,
                    time_out=self.cfg.data['unlock_time_out'],
                    max_torque=self.cfg.data['unlock_max_torque'],
                    unlock_angle=self.cfg.data['unlock_angle']
                    )
                rospy.logdebug(f"Final error after unlocking robot from plug: "
                               f"(Linear error={lin_ang_err[0]}[m] | Angular error={lin_ang_err[1]}[rad])")
                if sus_unl_plug:
                    sus_dec_plug, lin_ang_err = self.pilot.try2_decouple_to_plug(
                        time_out=self.cfg.data['decouple_time_out'],
                        max_force=self.cfg.data['decouple_max_force'],
                        max_torque=self.cfg.data['decouple_max_torque'],
                        decoupling_tolerance=self.cfg.data['decouple_tolerance']
                        )
                    rospy.logdebug("Final error after decoupling robot from plug: "
                                   f"(Linear error={lin_ang_err[0]}[m] | Angular error={lin_ang_err[1]}[rad])")
        rospy.loginfo(f"Unlock robot from plug successfully: {sus_unl_plug}")
        rospy.loginfo(f"Decoupling robot and plug successfully: {sus_dec_plug}")
        if not sus_unl_plug or not sus_dec_plug:
            raise StateMachineError(f"Spatial error to large. Robot is probably in an undefined condition.")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.plug_released, out.stop)
        return outcome
