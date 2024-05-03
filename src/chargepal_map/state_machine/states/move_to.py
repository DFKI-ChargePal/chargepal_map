""" This file implements the state >>MoveTo<< """
from __future__ import annotations
# libs
import rospy
import ur_pilot
import numpy as np
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


class MoveToStartLs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.stop, out.completed], 
                       input_keys=['job_id'],
                       output_keys=['job_id'])

    def execute(self, ud: Any) -> str:
        # Check job id
        job_id = ud.job_id
        if job_id != job_ids.move_home_arm:
             raise StateMachineError(f"Not a proper job ID '{job_id}'")
        start_pos = self.pilot.robot.joint_pos
        home_pos = self.cfg.data['home_joint_pos']
        # Try to avoid dangerous movements
        error_pos = np.abs(np.array(home_pos) - start_pos)
        if np.all(error_pos > self.cfg.data['max_moving_tolerance']):
            raise StateMachineError(f"Distance to home position is to large: {error_pos}. To dangerous to move ;)")
        with self.pilot.context.position_control():
            self.pilot.robot.movej(home_pos, self.cfg.data['vel'], self.cfg.data['acc'])
        # Get finale position and check remaining error to target
        finale_pos = self.pilot.robot.joint_pos
        error_pos = np.abs(np.array(home_pos) - finale_pos)
        if np.all(error_pos > 1e-2):
            raise StateMachineError(f"Remaining distance to home position is to large: {error_pos}. "
                                    f"Is the robot is running properly?")
        rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
        outcome = out.completed
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.completed, out.stop)
        return outcome


class MoveToStartRs(State): 

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.stop, out.completed], 
                       input_keys=['job_id'],
                       output_keys=['job_id'])
        
    def execute(self, ud: Any) -> str:
        # Check job id
        job_id = ud.job_id
        if job_id != job_ids.move_home_arm:
             raise StateMachineError(f"Not a proper job ID '{job_id}'")
        start_pos = self.pilot.robot.joint_pos
        home_pos = self.cfg.data['home_joint_pos']
        # Try to avoid dangerous movements
        error_pos = np.abs(np.array(home_pos) - start_pos)
        if np.all(error_pos > self.cfg.data['max_moving_tolerance']):
            raise StateMachineError(f"Distance to home position is to large: {error_pos}. To dangerous to move ;)")
        with self.pilot.context.position_control():
            self.pilot.robot.movej(home_pos, self.cfg.data['vel'], self.cfg.data['acc'])
        # Get finale position and check remaining error to target
        finale_pos = self.pilot.robot.joint_pos
        error_pos = np.abs(np.array(home_pos) - finale_pos)
        if np.all(error_pos > 1e-2):
            raise StateMachineError(f"Remaining distance to home position is to large: {error_pos}. "
                                    f"Is the robot is running properly?")
        rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
        outcome = out.completed
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.completed, out.stop)
        return outcome


class MoveToCompletion(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.stop, out.completed],
                       input_keys=['job_id'],
                       output_keys=['job_id'])

    def execute(self, ud: Any) -> str:
        print()
        job_id = ud.job_id
        wps = self.cfg.data[job_id]['joint_waypoints']
        rospy.loginfo(f"Start moving the arm to a save driving position.")
        with self.pilot.context.position_control():
            self.pilot.robot.move_path_j(wps, self.cfg.data['vel'], self.cfg.data['acc'])
        rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
        outcome = out.completed
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.completed, out.stop)
        return outcome


class MoveToPlugPreObs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, outcomes=[out.stop, out.plug_pre_obs], input_keys=['job_id'], output_keys=['job_id'])

    def execute(self, ud: Any) -> str:
        print()
        job_id = ud.job_id
        if job_id == job_ids.plug_out_ads_ac:
            rospy.loginfo(f"Start moving the arm to the adapter station on the left side")
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(self.cfg.data[job_id]['joint_waypoints'],
                                             self.cfg.data['vel'],
                                             self.cfg.data['acc'])
        elif job_id == job_ids.plug_out_ads_dc:
            rospy.loginfo(f"Start moving the arm to the adapter station on the right side")
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(self.cfg.data[job_id]['joint_waypoints'],
                                             self.cfg.data['vel'],
                                             self.cfg.data['acc'])
        elif job_id == job_ids.plug_out_bcs_ac:
            rospy.loginfo(f"Start moving the arm to the battery charging station on the left side")
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(self.cfg.data[job_id]['joint_waypoints'],
                                             self.cfg.data['vel'],
                                             self.cfg.data['acc'])
        else:
            raise StateMachineError(f"Invalid or undefined job ID '{job_id}' for this state.")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.plug_pre_obs, out.stop)
        return outcome


class MoveToSocketPreObs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.stop, out.socket_pre_obs], 
                       input_keys=['job_id'], 
                       output_keys=['job_id'])

    def execute(self, ud: Any) -> str:
        print()
        job_id = ud.job_id
        if job_id == job_ids.plug_in_ads_ac:
            rospy.loginfo(f"Start moving the plug to the adapter station on the left side")
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(self.cfg.data[job_id]['joint_waypoints'],
                                             self.cfg.data['vel'],
                                             self.cfg.data['acc'])
        elif job_id == job_ids.plug_in_ads_dc:
            rospy.loginfo(f"Start moving the plug to the adapter station on the right side")
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(self.cfg.data[job_id]['joint_waypoints'],
                                             self.cfg.data['vel'],
                                             self.cfg.data['acc'])
        elif job_id == job_ids.plug_in_bcs_ac:
            rospy.loginfo(f"Start moving the plug to the battery charging station on the left side")
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(self.cfg.data[job_id]['joint_waypoints'],
                                             self.cfg.data['vel'],
                                             self.cfg.data['acc'])
        else:
            raise StateMachineError(f"Invalid or undefined job ID '{job_id}' for this state.")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.socket_pre_obs, out.stop)
        return outcome


class MoveToPlugPreAttached(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.stop, out.plug_pre_attached], 
                       input_keys=['job_id', 'T_base2socket'], output_keys=['job_id', 'T_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start moving the arm to plug pre-attach pose')
        job_id = ud.job_id
        # Get transformation matrices
        if job_id in job_ids.plug_out():
            T_base2socket = ud.T_base2socket
        elif job_id in job_ids.plug_in():
            T_base2socket = sm.SE3().Rt(
                R=sm.SO3.EulerVec(self.cfg.data[job_id]['eulvec_base2socket']), 
                t=self.cfg.data[job_id]['trans_base2socket']
            )
            ud.T_base2socket = T_base2socket
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
                sus, _ = self.pilot.try2_move_to_plug_id_observation(T_base2socket)
                # Check with the ID whether the correct plug is in place or not.
                dtt_cfg_fp = self.cfg.data['detector'][self.cfg.data[plug_type]]
                # found, _ = self.pilot.find_target_pose(
                #     detector_fp=dtt_cfg_fp,
                #     time_out=self.cfg.data['time_out'])
                sus, _ = self.pilot.try2_approach_to_plug(T_base2socket)
        rospy.loginfo(f"Arm ended in pre-attached pose successfully: {sus}")
        rospy.logdebug(f"Transformation: Base-TCP = {ur_pilot.utils.se3_to_str(self.pilot.robot.tcp_pose)}")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.plug_pre_attached, out.stop)
        return outcome


class MoveToPlugPreConnected(State):

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
