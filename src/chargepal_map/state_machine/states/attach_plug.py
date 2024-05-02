""" This file implements the state >>AttachPlug<< """
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


class AttachPlug(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self,
                       outcomes=[out.stop, out.plug_attached],
                       input_keys=['job_id', 'T_base2socket'],
                       output_keys=['job_id', 'T_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start attaching the robot to the plug')
        job_id = ud.job_id
        # Get transformation matrices
        T_base2socket = sm.SE3(ud.T_base2socket)
        # Get plug type key
        if job_id in job_ids.type2_female():
            plug_type = 'type2_female'
        elif job_id in job_ids.type2_male():
            plug_type = 'type2_male'
        elif job_id in job_ids.ccs_female():
            plug_type = 'ccs_female'
        else:
            raise StateMachineError(f"Invalid or undefined job ID '{job_id}' for this state.")
        # Start attaching procedure
        with self.pilot.plug_model.context(plug_type):
            with self.pilot.context.force_control():
                sus_cup_plug, lin_ang_err = self.pilot.try2_couple_to_plug(
                    T_base2socket=T_base2socket,
                    time_out=self.cfg.data['couple_time_out'],
                    max_force=self.cfg.data['couple_max_force'],
                    max_torque=self.cfg.data['couple_max_torque'],
                    couple_tolerance=self.cfg.data['couple_tolerance']
                    )
                rospy.loginfo(f"Coupling robot and plug successfully: {sus_cup_plug}")
                rospy.logdebug(f"Final error after coupling robot and plug: "
                               f"(Linear error={lin_ang_err[0]}[m] | Angular error={lin_ang_err[1]}[rad])")
                if sus_cup_plug:
                    sus_lock_plug, lin_ang_err = self.pilot.try2_lock_plug(
                        T_base2socket=T_base2socket,
                        time_out=self.cfg.data['lock_time_out'],
                        max_torque=self.cfg.data['lock_max_torque'],
                        lock_angle=self.cfg.data['lock_angle']
                        )
                    rospy.loginfo(f"Lock robot with plug successfully: {sus_lock_plug}")
                    rospy.logdebug(f"Final error after locking robot with plug: "
                                   f"(Linear error={lin_ang_err[0]}[m] | Angular error={lin_ang_err[1]}[rad])")
        if not sus_cup_plug or not sus_lock_plug:
            raise StateMachineError(f"Spatial error to large. Robot is probably in an undefined condition.")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.plug_attached, out.stop)
        return outcome
