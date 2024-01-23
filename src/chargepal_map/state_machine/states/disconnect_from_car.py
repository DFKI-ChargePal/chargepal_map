from __future__ import annotations

# global
import time
import rospy
import cvpd as pd
import numpy as np
from smach import State
from time import perf_counter as _t_now
from rigmopy import Pose, Vector6d

# local
import chargepal_map.state_machine.outcomes as out
from chargepal_map.ui.user_client import UserClient
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any
from ur_pilot import Pilot


class MoveArmToCar(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.DisconnectFromCar.arm_in_car_obs])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move arm to car')
        rospy.logdebug(f"Car observation joint positions: {self.cfg.data['observation_joint_position']}")
        with self.pilot.position_control():
            self.pilot.move_to_joint_pos(self.cfg.data['observation_joint_position'])
        return self.uc.request_action(out.DisconnectFromCar.arm_in_car_obs, out.Common.stop)


class ObservePlugOnCar(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.DisconnectFromCar.arm_in_car_pre_connect],
                       output_keys=['xyz_xyzw_base2socket'])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Observe plug on car')
        # Create detector
        dtt_cfg_fp = self.cfg.data['detector_dir'].joinpath(self.cfg.data['detector_cfg'])
        dtt = pd.factory.create(dtt_cfg_fp)
        # Link camera
        if self.pilot.robot.cam is None:
            raise RuntimeError(f"No camera in ur-pilot registered. Observation not possible.")
        dtt.register_camera(self.pilot.robot.cam)

        # Search for ArUco pattern
        found = False
        # Use time out to exit loop
        _t_out = 5.0
        _t_start = _t_now()
        while _t_now() - _t_start <= _t_out and not found:
            # Give the robot some time to stop
            time.sleep(1.0)
            found, pose_cam2socket = dtt.find_pose()
            if found:
                # Search for transformation from base to socket
                # Get transformation matrices
                T_flange2cam = self.pilot.robot.cam_mdl.T_flange2camera
                T_base2flange = self.pilot.robot.get_pose().transformation
                T_cam2socket = Pose().from_xyz_xyzw(*pose_cam2socket).transformation
                # Apply transformation chain
                T_base2socket = T_base2flange @ T_flange2cam @ T_cam2socket
        if found:
            ud.xyz_xyzw_base2socket = T_base2socket.pose.xyz_xyzw
        else:
            raise RuntimeError(f"Can't find socket."
                               f"Make sure detector is proper set up and pattern is in camera view")
        return self.uc.request_action(out.DisconnectFromCar.arm_in_car_pre_connect, out.Common.stop)


class GraspPlugOnCar(State):

    _pose_socket2hook = Pose().from_xyz([0.0, 0.0, 0.034])
    _pose_socket2hook_pre = Pose().from_xyz([0.0, 0.0, 0.034 - 0.02]).from_axis_angle([0.0, 0.0, -np.pi/2])
    _pose_socket2hook_itm = Pose().from_xyz([0.0, 0.0, 0.034]).from_axis_angle([0.0, 0.0, -np.pi/2])

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_car_connect],
                       input_keys=['xyz_xyzw_base2socket'])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Grasp plug on car')
        # Get transformation matrices
        T_base2socket = Pose().from_xyz_xyzw(*ud.xyz_xyzw_base2socket).transformation
        T_socket2hook = self._pose_socket2hook.transformation
        T_socket2hook_pre = self._pose_socket2hook_pre.transformation
        T_socket2hook_itm = self._pose_socket2hook_itm.transformation
        # Apply transformation chain
        T_base2hook = T_base2socket @ T_socket2hook
        T_base2hook_pre = T_base2socket @ T_socket2hook_pre
        T_base2hook_itm = T_base2socket @ T_socket2hook_itm
        # Perform actions
        with self.pilot.position_control():
            # Move to pre-pose to hook up to plug
            self.pilot.move_to_tcp_pose(T_base2hook_pre.pose)
        with self.pilot.motion_control():
            self.pilot.move_to_tcp_pose(T_base2hook_itm.pose, time_out=5.0)
        pose_base2tcp = self.pilot.robot.get_tcp_pose()
        pose_base2tcp = pose_base2tcp.from_axis_angle(T_base2hook.pose.axis_angle)
        with self.pilot.position_control():
            self.pilot.move_to_tcp_pose(pose_base2tcp, time_out=5.0)

        return self.uc.request_action(out.DisconnectFromCar.plug_in_car_connect, out.Common.stop)


class RemovePlugFromCar(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_car_post_connect])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Remove plug from car')
        plug_out_ft = Vector6d().from_xyzXYZ([0.0, 0.0, -abs(self.cfg.data['force']), 0.0, 0.0, 0.0])
        with self.pilot.force_control():
            success = self.pilot.tcp_force_mode(
                wrench=plug_out_ft,
                compliant_axes=[0, 0, 1, 0, 0, 0],
                distance=self.cfg.data['moving_distance'],
                time_out=self.cfg.data['force_mode_time_out'])
        if not success:
            raise RuntimeError(f"Error while trying to unplug. Plug is probably still connected.")
        
        return self.uc.request_action(out.DisconnectFromCar.plug_in_car_post_connect, out.Common.stop)


class MovePlugToBattery(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_bat_obs])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move plug to battery')
        rospy.logdebug(f"Battery observation joint positions: {self.cfg.data['observation_joint_position']}")
        with self.pilot.position_control():
            self.pilot.move_to_joint_pos(self.cfg.data['observation_joint_position'])
        return self.uc.request_action(out.DisconnectFromCar.plug_in_bat_obs, out.Common.stop)


class ObserveSocketOnBattery(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_bat_pre_connect],
                       output_keys=['xyz_xyzw_base2socket'])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Observe socket on battery')
        # Create detector
        dtt_cfg_fp = self.cfg.data['detector_dir'].joinpath(self.cfg.data['detector_cfg'])
        dtt = pd.factory.create(dtt_cfg_fp)
        # Link camera
        if self.pilot.robot.cam is None:
            raise RuntimeError(f"No camera in ur-pilot registered. Observation not possible.")
        dtt.register_camera(self.pilot.robot.cam)

        # Search for ArUco pattern
        found = False
        # Use time out to exit loop
        _t_out = 5.0
        _t_start = _t_now()
        while _t_now() - _t_start <= _t_out and not found:
            # Give the robot some time to stop
            time.sleep(1.0)
            found, pose_cam2socket = dtt.find_pose()
            if found:
                # Search for transformation from base to socket
                # Get transformation matrices
                T_flange2cam = self.pilot.robot.cam_mdl.T_flange2camera
                T_base2flange = self.pilot.robot.get_pose().transformation
                T_cam2socket = Pose().from_xyz_xyzw(*pose_cam2socket).transformation
                # Apply transformation chain
                T_base2socket = T_base2flange @ T_flange2cam @ T_cam2socket
        if found:
            ud.xyz_xyzw_base2socket = T_base2socket.pose.xyz_xyzw
        else:
            raise RuntimeError(f"Can't find socket."
                               f"Make sure detector is proper set up and pattern is in camera view")
        return self.uc.request_action(out.DisconnectFromCar.plug_in_bat_pre_connect, out.Common.stop)


class InsertPlugToBattery(State):

    _pose_socket2socket_pre = Pose().from_xyz(xyz=[0.0, 0.0, 0.0 - 0.02])
    _pose_socket2fpi = Pose().from_xyz(xyz=[0.0, 0.0, 0.034])

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_bat_connect],
                       input_keys=['xyz_xyzw_base2socket'])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Insert plug to battery')
        # Get transformation matrices
        T_base2socket = Pose().from_xyz_xyzw(*ud.xyz_xyzw_base2socket).transformation
        T_socket2fpi = self._pose_socket2fpi.transformation
        T_socket2socket_pre = self._pose_socket2socket_pre.transformation
        # Apply transformation chain
        T_base2socket_pre = T_base2socket @ T_socket2socket_pre
        # Perform actions
        with self.pilot.position_control():    
                # Move to socket with some safety distance
                self.pilot.move_to_tcp_pose(T_base2socket_pre.pose)
        time.sleep(1.0)
        with self.pilot.force_control():
            self.pilot.one_axis_tcp_force_mode(axis='z', force=20.0, time_out=4.0)
            self.pilot.plug_in_force_ramp(f_axis='z', f_start=75.0, f_end=125, duration=4.0)
            self.pilot.relax(2.0)
            # Check if robot is in target area
            T_base2fpi = T_base2socket @ T_socket2fpi
            xyz_base2fpi_base_est = np.reshape(T_base2fpi.tau, 3)
            xyz_base2fpi_base_meas = np.reshape(self.pilot.robot.get_tcp_pose().xyz, 3)
            error = np.sqrt(np.sum(np.square(xyz_base2fpi_base_est - xyz_base2fpi_base_meas)))
            if error > 0.01:
                raise RuntimeError(f"Remaining position error {error} is to large. "
                                   f"Robot is probably in an undefined condition.")
        return self.uc.request_action(out.DisconnectFromCar.plug_in_bat_connect, out.Common.stop)


class ReleasePlugOnBattery(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.DisconnectFromCar.arm_in_bat_post_connect])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Release plug on battery')
        with self.pilot.position_control():
            # Rotate 90 deg counter clockwise to open the plug lock
            pose_tcp2target = Pose().from_axis_angle([0, 0, -np.pi/2])
            pose_base2tcp = self.pilot.robot.get_tcp_pose()
            pose_base2target = pose_base2tcp * pose_tcp2target
            self.pilot.move_to_tcp_pose(pose_base2target, time_out=3.0)
        with self.pilot.motion_control():
            # Move -20 mm in tcp z direction to disconnect from plug
            pose_tcp2target = Pose().from_xyz([0.0, 0.0, -0.025])
            pose_base2tcp = self.pilot.robot.get_tcp_pose()
            pose_base2target = pose_base2tcp * pose_tcp2target
            self.pilot.move_to_tcp_pose(pose_base2target, time_out=3.0)

        # Check if robot is in target area
        xyz_base2target_base_est = np.reshape(pose_base2target.xyz, 3)
        xyz_base2target_base_meas = np.reshape(self.pilot.robot.get_tcp_pose().xyz, 3)
        error = np.sqrt(np.sum(np.square(xyz_base2target_base_est - xyz_base2target_base_meas)))
        if error > 0.01:
            raise RuntimeError(f"Remaining position error {error} is to large. "
                               f"Robot is probably in an undefined condition.")

        return self.uc.request_action(out.DisconnectFromCar.arm_in_bat_post_connect, out.Common.stop)


class MoveArmToDrivePos(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.DisconnectFromCar.arm_in_driving_pose])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move arm to drive pos')
        rospy.logdebug(f"Driving joint positions: {self.cfg.data['drive_joint_position']}")
        with self.pilot.position_control():
            self.pilot.move_to_joint_pos(self.cfg.data['drive_joint_position'])
        return self.uc.request_action(out.DisconnectFromCar.arm_in_driving_pose, out.Common.stop)
