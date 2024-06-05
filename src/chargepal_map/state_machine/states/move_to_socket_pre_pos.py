""" This file implements the state >>MoveToSocketPrePos<< """
from __future__ import annotations
# libs
import rospy
import ur_pilot
from smach import State
import spatialmath as sm

from chargepal_map.core import job_ids
from chargepal_map.state_machine.outcomes import out
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.utils import StateMachineError

# typing
from typing import Any
from ur_pilot import Pilot


class MoveToSocketPrePos(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, outcomes=[out.stop, out.plug_pre_connected], 
                       input_keys=['job_id', 'T_base2socket'], 
                       output_keys=['job_id', 'T_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start moving the plug to the pre connecting pose')
        job_id = ud.job_id
        # Get transformation matrices
        if job_id in job_ids.plug_in():
            T_base2socket = ud.T_base2socket
        elif job_id in job_ids.plug_out():
            T_base2socket = sm.SE3().Rt(
                R=sm.SO3.EulerVec(self.cfg.data[job_id]['eulvec_base2socket']), 
                t=self.cfg.data[job_id]['trans_base2socket']
            )
            ud.T_base2socket = T_base2socket
            # Move in a save path to the battery
            if job_id == job_ids.plug_out_ads_ac:
                rospy.loginfo(f"Start moving the arm to the battery comming from the adapter station on the left side")
                with self.pilot.context.position_control():
                    self.pilot.robot.move_path_j(self.cfg.data[job_id]['joint_waypoints'],
                                                self.cfg.data['vel'],
                                                self.cfg.data['acc'])
            elif job_id == job_ids.plug_out_ads_dc:
                rospy.loginfo(f"Start moving the arm to the battery comming from the adapter station on the right side")
                with self.pilot.context.position_control():
                    self.pilot.robot.move_path_j(self.cfg.data[job_id]['joint_waypoints'],
                                                self.cfg.data['vel'],
                                                self.cfg.data['acc'])
            elif job_id == job_ids.plug_out_bcs_ac:
                rospy.loginfo(f"Start moving the arm to the battery comming from the battery charging station on the left side")
                with self.pilot.context.position_control():
                    self.pilot.robot.move_path_j(self.cfg.data[job_id]['joint_waypoints'],
                                                self.cfg.data['vel'],
                                                self.cfg.data['acc'])
            else:
                raise StateMachineError(f"Invalid or undefined job ID '{job_id}' for this state.")

        else:
            raise StateMachineError(f"Invalid or undefined job ID '{job_id}' for this state.")
        # Get plug type key
        if job_id in job_ids.type2_female():
            plug_type = 'type2_female'
        elif job_id in job_ids.type2_male():
            plug_type = 'type2_male'
        elif job_id in job_ids.ccs_female():
            plug_type = 'ccs_female'
        else:
            raise StateMachineError(f"Invalid or undefined job ID '{job_id}' for this state.")
        with self.pilot.plug_model.context(plug_type):
            with self.pilot.context.position_control():
                sus, _ = self.pilot.try2_approach_to_socket(T_base2socket)
        rospy.loginfo(f"Arm ended in pre-insert pose successfully: {sus}")
        rospy.logdebug(f"Transformation: Base-TCP = {ur_pilot.utils.se3_to_str(self.pilot.robot.tcp_pose)}")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.plug_pre_connected, out.stop)
        return outcome
