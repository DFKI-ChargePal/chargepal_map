""" This file implements the state >>ReleasePlug<< """
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


class ReleasePlug(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[
                           out.plug_released,
                           out.err_plug_in_recover,
                           out.err_plug_in_stop,
                           out.job_stopped], 
                       input_keys=['job', 'battery_id'],
                       output_keys=['job', 'battery_id'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Get user and configuration data
        job: Job = ud.job
        cfg_data = self.cfg.extract_data(ud.battery_id)
        # Get transformation matrix of socket pose
        if job.in_progress_mode():
            exterior = job.is_part_of_plug_in()
            interior = job.is_part_of_plug_out()
        elif job.in_recover_mode():
            exterior = job.is_part_of_plug_out()
            interior = job.is_part_of_plug_in()
        else:
            raise StateMachineError(f"Job in an invalid mode. Interrupt process")
        if exterior:
            T_base2socket = job.exterior_socket.T_base2socket_close_up
        elif interior:
            T_base2socket = job.interior_socket.T_base2socket_close_up
        else:
            raise StateMachineError(f"Invalid or undefined job '{job}' for this state")
        outcome = ''
        if self.user_cb is not None:
            rospy.loginfo(f"Ready to release plug from arm")
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        if outcome != out.job_stopped:
            rospy.loginfo('Start to try releasing the arm from the plug')
            with self.pilot.plug_model.context(plug_type=job.get_plug_type()):
                sus_unl_plug, sus_dec_plug = False, False
                with self.pilot.context.force_control():
                    sus_unl_plug, lin_ang_err = self.pilot.try2_unlock_plug(
                        T_base2socket=T_base2socket,
                        time_out=cfg_data['unlock_time_out'],
                        max_torque=cfg_data['unlock_max_torque'],
                        unlock_angle=cfg_data['unlock_angle']
                        )
                    rospy.logdebug(f"Final error after unlocking robot from plug: "
                                f"(Linear error={lin_ang_err[0]}[m] | Angular error={lin_ang_err[1]}[rad])")
                    if sus_unl_plug:
                        sus_dec_plug, lin_ang_err = self.pilot.try2_decouple_to_plug(
                            time_out=cfg_data['decouple_time_out'],
                            max_force=cfg_data['decouple_max_force'],
                            max_torque=cfg_data['decouple_max_torque'],
                            decouple_tolerance=cfg_data['decouple_tolerance']
                            )
                        rospy.logdebug("Final error after decoupling robot from plug: "
                                    f"(Linear error={lin_ang_err[0]}[m] | Angular error={lin_ang_err[1]}[rad])")
            rospy.loginfo(f"Unlock robot from plug successfully: {sus_unl_plug}")
            rospy.loginfo(f"Decoupling robot and plug successfully: {sus_dec_plug}")
            if sus_unl_plug and sus_dec_plug:
                if job.in_progress_mode():
                    outcome = out.plug_released
                elif job.in_recover_mode():
                    outcome = out.err_plug_in_recover
                else:
                    raise StateMachineError(f"Job in an invalid mode. Interrupt process")
            else:
                job.enable_stop_mode()
                outcome = out.err_plug_in_stop
                rospy.logerr(f"Robot was not able to release the plug successfully. Arm is probably still connected")
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
