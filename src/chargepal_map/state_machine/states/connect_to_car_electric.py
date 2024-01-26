""" 
    State implementations to connect from battery to the car/adapter station
    using the plug with the electrical lock.
"""
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


class MoveArmToBattery(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCarElectric.arm_in_bat_pre_obs])
        
    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start moving the arm to battery observation pose')
        rospy.logdebug(f"Battery observation joint positions: {self.cfg.data['observation_joint_position']}")
        with self.pilot.position_control():
            self.pilot.move_to_joint_pos(self.cfg.data['observation_joint_position'])
        rospy.loginfo(f"Arm ended in battery observation pose: Base-TCP = {self.pilot.robot.get_tcp_pose()}")
        return self.uc.request_action(out.ConnectToCarElectric.arm_in_bat_pre_obs, out.Common.stop)


class ObservePlugOnBattery(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCarElectric.arm_in_bat_post_obs],
                       output_keys=['xyz_xyzw_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start observing the socket with plug on battery')
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
        rospy.loginfo(f"Found socket pose: Base-Socket = {T_base2socket.pose}")
        return self.uc.request_action(out.ConnectToCarElectric.arm_in_bat_post_obs, out.Common.stop)


class MoveArmToBatteryPreGrasp(State):

    _pose_socket2save_pre = Pose().from_xyz([0.005, 0.0, 0.034 - 0.115])
    
    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCarElectric.arm_in_bat_pre_connect], 
                       input_keys=['xyz_xyzw_base2socket'],
                       output_keys=['xyz_xyzw_base2socket'])

    def execute(self, ud):
        print(), rospy.loginfo('Start moving the arm to pre-grasp pose on battery')
        # Get transformation matrices
        T_base2socket = Pose().from_xyz_xyzw(*ud.xyz_xyzw_base2socket).transformation
        T_socket2save_pre = self._pose_socket2save_pre.transformation
        # Apply transformation chain
        T_base2save_pre = T_base2socket @ T_socket2save_pre
        # Perform actions
        with self.pilot.position_control():
            # Move to pre-pose to hook up to plug
            self.pilot.move_to_tcp_pose(T_base2save_pre.pose)
        rospy.loginfo(f"Arm ended in pre-grasp pose: Base-TCP = {self.pilot.robot.get_tcp_pose()}")
        return self.uc.request_action(out.ConnectToCarElectric.arm_in_bat_pre_connect, out.Common.stop)


class GraspPlugOnBattery(State):

    _pose_socket2fpi = Pose().from_xyz(xyz=[0.0, 0.0, 0.034])

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCarElectric.plug_in_bat_connect],
                       input_keys=['xyz_xyzw_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start grasping the plug on battery')
        # Get transformation matrices
        T_socket2fpi = self._pose_socket2fpi.transformation
        T_base2socket = Pose().from_xyz_xyzw(*ud.xyz_xyzw_base2socket).transformation
        # Perform actions
        with self.pilot.force_control():
            with self.pilot.open_plug_connection():
                self.pilot.one_axis_tcp_force_mode(axis='z', force=20.0, time_out=7.0)
            # Check if robot is in target area
            T_base2fpi = T_base2socket @ T_socket2fpi
            xyz_base2fpi_base_est = np.reshape(T_base2fpi.tau, 3)
            xyz_base2fpi_base_meas = np.reshape(self.pilot.robot.get_tcp_pose().xyz, 3)
            error = np.sqrt(np.sum(np.square(xyz_base2fpi_base_est - xyz_base2fpi_base_meas)))
            if error > 0.0075:
                raise RuntimeError(f"Remaining position error {error} is to large. "
                                   f"Robot is probably in an undefined condition.")
        rospy.loginfo(f"Arm connected to the plug with a residual position error of {1000 * error} mm")
        return self.uc.request_action(out.ConnectToCarElectric.plug_in_bat_connect, out.Common.stop)


class RemovePlugFromBattery(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCarElectric.plug_in_bat_post_connect])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start removing the plug from battery')
        plug_out_ft = Vector6d().from_xyzXYZ([0.0, 0.0, -abs(self.cfg.data['force']), 0.0, 0.0, 0.0])
        with self.pilot.force_control():
            success = self.pilot.tcp_force_mode(
                wrench=plug_out_ft,
                compliant_axes=[0, 0, 1, 0, 0, 0],
                distance=self.cfg.data['moving_distance'],
                time_out=self.cfg.data['force_mode_time_out'])
        if not success:
            raise RuntimeError(f"Error while trying to unplug. Plug is probably still connected.")
        rospy.loginfo(f"Plug successfully removed from the battery socket.")
        return self.uc.request_action(out.ConnectToCarElectric.plug_in_bat_post_connect, out.Common.stop)


class MovePlugToCar(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCarElectric.plug_in_car_pre_obs])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start moving the plug to the car')
        rospy.logdebug(f"Car observation joint positions: {self.cfg.data['observation_joint_position']}")
        with self.pilot.position_control():
            self.pilot.move_to_joint_pos(self.cfg.data['observation_joint_position'])
        rospy.loginfo(f"Plug ended in car observation pose: Base-TCP = {self.pilot.robot.get_tcp_pose()}")
        return self.uc.request_action(out.ConnectToCarElectric.plug_in_car_pre_obs, out.Common.stop)


class ObserveSocketOnCar(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCarElectric.plug_in_car_post_obs],
                       output_keys=['xyz_xyzw_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start observing the socket on the car')
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
        rospy.loginfo(f"Found socket pose: Base-Socket = {T_base2socket.pose}")
        return self.uc.request_action(out.ConnectToCarElectric.plug_in_car_post_obs, out.Common.stop)


class MovePlugToCarPreConnect(State):

    _pose_socket2save_pre = Pose().from_xyz([0.0, 0.0, -0.02])
    
    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCarElectric.plug_in_car_pre_connect], 
                       input_keys=['xyz_xyzw_base2socket'],
                       output_keys=['xyz_xyzw_base2socket'])

    def execute(self, ud):
        print(), rospy.loginfo('Start moving the plug to pre-insert pose on the car')
        # Get transformation matrices
        T_base2socket = Pose().from_xyz_xyzw(*ud.xyz_xyzw_base2socket).transformation
        T_socket2save_pre = self._pose_socket2save_pre.transformation
        # Apply transformation chain
        T_base2save_pre = T_base2socket @ T_socket2save_pre
        # Perform actions
        with self.pilot.position_control():
            # Move to pre-pose to hook up to plug
            self.pilot.move_to_tcp_pose(T_base2save_pre.pose)
        rospy.loginfo(f"Plug ended in pre-insert pose: Base-TCP = {self.pilot.robot.get_tcp_pose()}")
        return self.uc.request_action(out.ConnectToCarElectric.plug_in_car_pre_connect, out.Common.stop)


class InsertPlugToCar(State):

    _pose_socket2socket_touch = Pose().from_xyz(xyz=[-0.01, 0.0, 0.005])
    _pose_socket2socket_align = Pose().from_xyz(xyz=[0.0, 0.0, 0.01])
    _pose_socket2fpi = Pose().from_xyz(xyz=[0.0, 0.0, 0.034])

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCarElectric.plug_in_car_connect],
                       input_keys=['xyz_xyzw_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start inserting the plug to the car')
        # Get transformation matrices
        T_base2socket = Pose().from_xyz_xyzw(*ud.xyz_xyzw_base2socket).transformation
        T_socket2socket_touch = self._pose_socket2socket_touch.transformation
        T_socket2socket_align = self._pose_socket2socket_align.transformation
        T_socket2fpi = self._pose_socket2fpi.transformation
        # Perform actions
        T_base2socket_touch = T_base2socket @ T_socket2socket_touch
        T_base2socket_align = T_base2socket @ T_socket2socket_align
        with self.pilot.motion_control():
            self.pilot.move_to_tcp_pose(T_base2socket_touch.pose, time_out=3.0)
            self.pilot.move_to_tcp_pose(T_base2socket_align.pose, time_out=3.0)
            # Check if robot is in target area
            xyz_base2sa_base_est = np.reshape(T_base2socket_align.tau, 3)
            xyz_base2sa_base_meas = np.reshape(self.pilot.robot.get_tcp_pose().xyz, 3)
            error = np.sqrt(np.sum(np.square(xyz_base2sa_base_est - xyz_base2sa_base_meas)))
            if error > 0.01:
                raise RuntimeError(f"Remaining position error {error} to alignment state is to large. "
                                   f"Robot is probably in an undefined condition.")
        with self.pilot.force_control():
            self.pilot.plug_in_force_ramp(f_axis='z', f_start=50.0, f_end=90, duration=3.0)
            self.pilot.relax(1.0)
            # Check if robot is in target area
            T_base2fpi = T_base2socket @ T_socket2fpi
            xyz_base2fpi_base_est = np.reshape(T_base2fpi.tau, 3)
            xyz_base2fpi_base_meas = np.reshape(self.pilot.robot.get_tcp_pose().xyz, 3)
            error = np.sqrt(np.sum(np.square(xyz_base2fpi_base_est - xyz_base2fpi_base_meas)))
            if error > 0.01:
                raise RuntimeError(f"Remaining position error {error} to fully plugged in state is to large. "
                                   f"Robot is probably in an undefined condition.")
        rospy.loginfo(f"Plug inserted in car with a residual position error of {1000 * error} mm")
        return self.uc.request_action(out.ConnectToCarElectric.plug_in_car_connect, out.Common.stop)


class ReleasePlugOnCar(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCarElectric.arm_in_car_post_connect])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start releasing the arm from the plug on the car')
        # Move -115 mm in tcp z direction to disconnect from plug
        pose_tcp2target = Pose().from_xyz([0.0, 0.0, -0.125])
        pose_base2tcp = self.pilot.robot.get_tcp_pose()
        pose_base2target = pose_base2tcp * pose_tcp2target
        with self.pilot.force_control():
            with self.pilot.open_plug_connection():
                self.pilot.one_axis_tcp_force_mode(axis='z', force=20.0, time_out=1.0)
                self.pilot.relax(0.5)
                self.pilot.one_axis_tcp_force_mode(axis='z', force=-20.0, time_out=2.0)
        with self.pilot.motion_control():
            self.pilot.move_to_tcp_pose(pose_base2target, time_out=3.0)

        # Check if robot is in target area
        xyz_base2target_base_est = np.reshape(pose_base2target.xyz, 3)
        xyz_base2target_base_meas = np.reshape(self.pilot.robot.get_tcp_pose().xyz, 3)
        error = np.sqrt(np.sum(np.square(xyz_base2target_base_est - xyz_base2target_base_meas)))
        if error > 0.01:
            raise RuntimeError(f"Remaining position error {error} is to large. "
                               f"Robot is probably in an undefined condition.")
        rospy.loginfo(f"Arm successfully released from the plug on the car.")
        return self.uc.request_action(out.ConnectToCarElectric.arm_in_car_post_connect, out.Common.stop)


class MoveArmToDrivePos(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCarElectric.arm_in_driving_pose])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start moving the arm to drive configuration')
        rospy.logdebug(f"Driving joint positions: {self.cfg.data['drive_joint_position']}")
        with self.pilot.position_control():
            self.pilot.move_to_joint_pos(self.cfg.data['drive_joint_position'])
        rospy.loginfo(f"Arm ended in drive pose: Base-TCP = {self.pilot.robot.get_tcp_pose()}")
        return self.uc.request_action(out.ConnectToCarElectric.arm_in_driving_pose, out.Common.stop)
