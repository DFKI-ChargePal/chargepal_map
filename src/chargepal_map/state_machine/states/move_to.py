""" This file implements the state >>MoveTo<< """
from __future__ import annotations
# libs
import rospy
import ur_pilot
import numpy as np
from smach import State
import spatialmath as sm

from chargepal_map.core import job_ids
from chargepal_map.ui.user_client import UserClient
from chargepal_map.state_machine.outcomes import out
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any
from ur_pilot import Pilot


class MoveToWs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
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
            self.pilot.robot.move_path_j(self.cfg.data['joint_waypoints'], self.cfg.data['vel'], self.cfg.data['acc'])
        rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
        return self.uc.request_action(out.completed, out.stop)


class MoveToPlugPreObs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
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
            raise ValueError(f"Invalid or undefined job ID '{job_id}' for this state.")
        return self.uc.request_action(out.plug_pre_obs, out.stop)


class MoveToSocketPreObs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
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
            raise ValueError(f"Invalid or undefined job ID '{job_id}' for this state.")
        return self.uc.request_action(out.socket_pre_obs, out.stop)


class MoveToPlugPreAttached(State):

    _T_plug2pre_attach = sm.SE3().Rt(R=sm.SO3.EulerVec((0.0, 0.0, -np.pi/2)), t=(0.0, 0.0, -0.02))

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.stop, out.plug_pre_attached], 
                       input_keys=['job_id', 'T_base2plug'], output_keys=['job_id', 'T_base2plug'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start moving the arm to plug pre-attach pose')
        job_id = ud.job_id
        # Get transformation matrices
        if job_id in job_ids.plug_in():
            T_base2plug = sm.SE3().Rt(
                R=sm.SO3.EulerVec(self.cfg.data['eulvec_base2plug']), 
                t=self.cfg.data['trans_base2plug']
            )
        elif job_id in job_ids.plug_out():
            T_base2plug = ud.T_base2plug
        else:
            raise ValueError(f"Invalid or undefined job ID '{job_id}' for this state.")
        T_plug2pre_attach = self._T_plug2pre_attach
        # Apply transformation chain
        T_base2pre_attach = T_base2plug * T_plug2pre_attach
        # Perform actions
        with self.pilot.context.position_control():
            # Move to the plug pre-attach pose
            self.pilot.move_to_tcp_pose(T_base2pre_attach)
        rospy.loginfo(f"Arm ended in pre-attach pose: "
                      f"Base-TCP = {ur_pilot.utils.se3_to_str(self.pilot.robot.tcp_pose)}")
        return self.uc.request_action(out.plug_pre_attached, out.stop)


class MoveToPlugPreConnected(State):

    _T_socket2pre_connect = sm.SE3().Trans([0.0, 0.0, 0.0 - 0.02])

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
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
                R=sm.SO3.EulerVec(self.cfg.data['eulvec_base2socket']), 
                t=self.cfg.data['trans_base2socket']
            )
        else:
            raise ValueError(f"Invalid or undefined job ID '{job_id}' for this state.")
        T_socket2pre_connect = self._T_socket2pre_connect
        # Apply transformation chain
        T_base2pre_connect = T_base2socket * T_socket2pre_connect
        # Performe actions
        with self.pilot.context.position_control():
            self.pilot.move_to_tcp_pose(T_base2pre_connect)
        rospy.loginfo(f"Plug ended in pre-insert pose: "
                      f"Base-TCP = {ur_pilot.utils.se3_to_str(self.pilot.robot.tcp_pose)}")
        return self.uc.request_action(out.plug_pre_connected, out.stop)
