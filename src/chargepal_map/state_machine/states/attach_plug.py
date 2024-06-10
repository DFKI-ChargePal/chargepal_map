""" This file implements the state >>AttachPlug<< """
from __future__ import annotations
# libs
import rospy
from smach import State
import spatialmath as sm

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


class AttachPlug(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self,
                       outcomes=[out.plug_attached,
                                 out.err_plug_out_retry,
                                 out.err_plug_out_recover,
                                 out.err_plug_in_stop, 
                                 out.job_stopped],
                       input_keys=['job', 'T_base2socket'],
                       output_keys=['job', 'T_base2socket'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self))) 
        # Get user and configuration data
        job: Job = ud.job
        T_base2socket = sm.SE3(ud.T_base2socket)
        rospy.loginfo('Start to try attaching the robot to the plug')
        sus_cup_plug, sus_dec_plug, sus_lock_plug = False, False, False
        # Start attaching procedure
        with self.pilot.plug_model.context(plug_type=job.get_plug_type()):
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
                else:
                    sus_dec_plug, lin_ang_err = self.pilot.try2_decouple_to_plug(
                        time_out=self.cfg.data['decouple_time_out'],
                        max_force=self.cfg.data['decouple_max_force'],
                        max_torque=self.cfg.data['decouple_max_torque'],
                        decoupling_tolerance=self.cfg.data['decouple_tolerance']
                        )
        if sus_cup_plug and sus_lock_plug:
            outcome = out.plug_attached
        elif not sus_cup_plug and not sus_lock_plug and sus_dec_plug:
            if job.retry_count > 1:
                outcome = out.err_obs_plug_recover
                rospy.loginfo(f"Robot was not able to attach to the plug. Try to recover the arm")
            else:
                job.retry()
                outcome = out.err_plug_out_retry
                rospy.loginfo(f"Robot was not able to attach to the plug. Retry attachment procedure")
        elif not sus_cup_plug and not sus_dec_plug or not sus_lock_plug:
            outcome = out.err_plug_in_stop
            rospy.logerr(f"Robot got stuck while trying to attach. Arm is in an undefined condition.")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.outcome, out.job_stopped)
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
