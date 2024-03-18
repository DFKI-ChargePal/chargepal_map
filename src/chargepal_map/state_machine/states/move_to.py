""" This file implements the state >>MoveTo<< """
from __future__ import annotations
# libs
import rospy
import ur_pilot
import numpy as np
from smach import State

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
        State.__init__(self, outcomes=[out.stop, out.arm_in_ws_ls])

    def execute(self, ud: Any) -> str:
        print()
        # Check if robot arm is in left side operation mode
        shoulder_pan_pos = self.pilot.robot.joint_pos[0]
        shoulder_pan_ws_ls = self.cfg.date.sd_pan_ws_ls
        is_ws_ls = True if shoulder_pan_ws_ls - np.pi/2 < shoulder_pan_pos < shoulder_pan_ws_ls + np.pi/2 else False
        if is_ws_ls:
            rospy.loginfo('Start moving the arm to a save pose in the workspace on the left side')
            with self.pilot.context.position_control():
                self.pilot.robot.move_path_j(self.cfg.data['joint_waypoints'], self.cfg.data['vel'], self.cfg.data['acc'])
        else:
            raise RuntimeError(f"Arm is not operating in the left side workspace. Movement is not allowed!")
        rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
        return self.uc.request_action(out.arm_in_ws_ls, out.stop)


# class MoveToWsRs(State):
    
#     def __init__(self, config: dict[str, Any], pilot: Pilot):
#         self.pilot = pilot
#         self.cfg = StateConfig(type(self), config=config)
#         self.uc = UserClient(self.cfg.data['step_by_user'])
#         State.__init__(self, outcomes=[out.stop, out.arm_in_ws_rs])

#     def execute(self, ud: Any) -> str:
#         print()
#         # Check if robot arm is in right side operation mode
#         shoulder_pan_pos = self.pilot.robot.joint_pos[0]
#         shoulder_pan_ws_rs = self.cfg.data.sd_pan_ws_rs
#         is_ws_rs = True if shoulder_pan_ws_rs - np.pi/2 < shoulder_pan_pos < shoulder_pan_ws_rs + np.pi/2 else False
#         if is_ws_rs:
#             rospy.loginfo('Start moving the arm to a save pose in the workspace on the right side')
#             with self.pilot.context.position_control():
#                 self.pilot.robot.move_path_j(self.cfg.data['joint_waypoints'], self.cfg.data['vel'], self.cfg.data['acc'])
#         else:
#             raise RuntimeError(f"Arm is not operating in the right side workspace. Movement is not allowed!")
#         rospy.loginfo(f"Arm ended in joint configuration: {ur_pilot.utils.vec_to_str(self.pilot.robot.joint_pos)}")
#         return self.uc.request_action(out.arm_in_ws_rs, out.stop)


class MoveToPlugPreObs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.stop, out.plug_pre_obs])

    def execute(self, ud: Any) -> str:
        print()
        return self.uc.request_action(out.plug_pre_obs, out.stop)

class MoveToSocketPreObs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.stop, out.socket_pre_obs])

    def execute(self, ud: Any) -> str:
        print()
        return self.uc.request_action(out.socket_pre_obs, out.stop)


class MoveToPlugPreAttached(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.stop, out.plug_pre_attached])

    def execute(self, ud: Any) -> str:
        print()
        return self.uc.request_action(out.plug_pre_attached, out.stop)


class MoveToPlugPreConnected(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.stop, out.plug_pre_connected])

    def execute(self, ud: Any) -> str:
        print()
        return self.uc.request_action(out.plug_pre_connected, out.stop)
