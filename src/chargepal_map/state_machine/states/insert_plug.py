""" This file implements the state >>InsertPlug<< """
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


class InsertPlug(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[
                           out.plug_connected,
                           out.err_plug_in_battery_retry,
                           out.err_plug_in_periphery_retry,
                           out.err_plug_in_recover,
                           out.err_plug_in_stop,
                           out.job_stopped,
                           ],
                       input_keys=['job'],
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Get user and configuration data
        job: Job = ud.job
        # Get transformation matrix of socket pose
        if job.in_progress_mode() or job.in_retry_mode():
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
            rospy.loginfo(f"Ready to insert the plug to the socket")
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        if outcome != out.job_stopped:
            rospy.loginfo('Start inserting the plug to the socket')
            with self.pilot.plug_model.context(plug_type=job.get_plug_type()): 
                sus_eng_plug, sus_ins_plug, sus_rmv_plug = False, False, False
                with self.pilot.context.force_control():
                    sus_eng_plug, lin_ang_err = self.pilot.try2_engage_with_socket(
                        T_base2socket=T_base2socket,
                        time_out=self.cfg.data['engage_time_out'],
                        max_force=self.cfg.data['engage_max_force'],
                        engage_depth=self.cfg.data['engage_depth'],
                        engage_tolerance=self.cfg.data['engage_tolerance']
                        )
                    rospy.loginfo(f"Engaging plug to socket successfully: {sus_eng_plug}")
                    rospy.logdebug(f"Final error after engaging between plug and socket: "
                                f"(Linear error={lin_ang_err[0]}[m] | Angular error={lin_ang_err[1]}[rad])")
                    if sus_eng_plug:
                        sus_ins_plug, lin_ang_err = self.pilot.try2_insert_plug(
                            T_base2socket=T_base2socket,
                            time_out=self.cfg.data['insert_time_out'],
                            start_force=self.cfg.data['insert_start_force'],
                            end_force=self.cfg.data['insert_end_force'],
                            insert_tolerance=self.cfg.data['insert_tolerance']
                            )
                        rospy.loginfo(f"Inserting plug to socket successfully: {sus_ins_plug}")
                        rospy.logdebug(f"Final error after inserting plug to socket: "
                                    f"(Linear error={lin_ang_err[0]}[m] | Angular error={lin_ang_err[1]}[rad])")
                    if not sus_eng_plug or not sus_ins_plug:
                        rospy.loginfo(f"Error while inserting the plug. Try to recover by removing the plug.")
                        sus_rmv_plug, lin_ang_err = self.pilot.try2_remove_plug(
                            time_out=self.cfg.data['remove_time_out'],
                            max_force=self.cfg.data['remove_max_force'],
                            remove_tolerance=self.cfg.data['remove_tolerance']
                        )
                        rospy.loginfo(f"Removing plug from socket successfully: {sus_rmv_plug}")
            
            if job.in_progress_mode() or job.in_retry_mode():
                if sus_eng_plug and sus_ins_plug:
                    job.enable_progress_mode()
                    outcome = out.plug_connected
                    rospy.loginfo(f"Robot connected plug and socket successfully")
                elif not sus_ins_plug and sus_rmv_plug:
                    if job.retry_count >= self.cfg.data['number_of_retries']:
                        job.enable_recover_mode()
                        outcome = out.err_plug_in_recover
                        rospy.loginfo(f"Robot was not able to connect plug to socket. Try to recover the arm")
                    else:
                        job.enable_retry_mode()
                        if job.is_part_of_plug_in():
                            outcome = out.err_plug_in_periphery_retry
                        elif job.is_part_of_plug_out():
                            outcome = out.err_plug_in_battery_retry
                        else:
                            raise StateMachineError(f"Invalid or undefined job '{job}' for this state")
                        rospy.loginfo(f"Robot was not able to connect plug to socket. Retry inserting procedure")
                elif not sus_eng_plug and not sus_ins_plug and not sus_rmv_plug:
                    job.enable_stop_mode()
                    outcome = out.err_plug_in_stop
                    rospy.logerr(f"Robot got stuck while trying to insert the plug. Arm is in an undefined condition.")
                else:
                    raise StateMachineError(f"Logical error. This situation should not occur. Check your implementation!")
            elif job.in_recover_mode():
                if sus_eng_plug and sus_ins_plug:
                    outcome = out.plug_connected
                    rospy.loginfo(f"Robot connected plug and socket successfully")
                else:
                    job.enable_stop_mode()
                    outcome = out.err_plug_in_stop
            else:
                raise StateMachineError(f"Job in an invalid mode. Interrupt process")
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
