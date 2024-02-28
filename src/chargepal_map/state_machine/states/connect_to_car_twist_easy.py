""" 
    State implementations to connect from battery to the car/adapter station
    using the plug with the mechanical twist lock.
"""
from __future__ import annotations

# libs
import time
import rospy
import ur_pilot
import cvpd as pd
import numpy as np
import spatialmath as sm

from smach import State
from time import perf_counter as _t_now

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
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCarTwist.arm_in_bat_pre_obs])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start moving the arm to battery observation pose')
        rospy.logdebug(f"Battery observation joint positions: {self.cfg.data['observation_joint_position']}")
        with self.pilot.context.position_control():
            self.pilot.move_to_joint_pos(q=self.cfg.data['observation_joint_position'])
        rospy.loginfo(f"Arm ended in battery observation pose: "
                      f"Base-TCP = {ur_pilot.utils.se3_to_str(self.pilot.robot.tcp_pose)}")
        return self.uc.request_action(out.ConnectToCarTwist.arm_in_bat_pre_obs, out.Common.stop)


class MoveArmToBatteryPreGrasp(State):

    _T_socket2pre_grasp = sm.SE3().Rt(R=sm.SO3.EulerVec((0.0, 0.0, -np.pi/2)), t=(0.005, 0.0, 0.034 - 0.02))

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCarTwist.arm_in_bat_pre_connect])

    def execute(self, ud):
        print(), rospy.loginfo('Start moving the arm to pre-grasp pose on battery')
        # Get transformation matrices
        T_base2socket = sm.SE3().Rt(
            R=sm.SO3.EulerVec(self.cfg.data['eulvec_base2socket']), 
            t=self.cfg.data['trans_base2socket']
            )
        T_socket2pre_grasp = self._T_socket2pre_grasp
        # Apply transformation chain
        T_base2pre_grasp = T_base2socket * T_socket2pre_grasp
        # Perform actions
        with self.pilot.context.position_control():
            # Move to pre-pose to hook up to plug
            self.pilot.move_to_tcp_pose(T_base2pre_grasp)
        rospy.loginfo(f"Arm ended in pre-grasp pose: "
                      f"Base-TCP = {ur_pilot.utils.se3_to_str(self.pilot.robot.tcp_pose)}")
        return self.uc.request_action(out.ConnectToCarTwist.arm_in_bat_pre_connect, out.Common.stop)


class GraspPlugOnBattery(State):

    _T_socket2fpi = sm.SE3().Trans([0.0, 0.0, 0.034])
    _T_socket2fpi_twist = sm.SE3().Rt(R=sm.SO3.EulerVec((0.0, 0.0, -np.pi/2)), t=(0.0, 0.0, 0.034))

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self,  outcomes=[out.Common.stop, out.ConnectToCarTwist.plug_in_bat_connect])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start grasping the plug on battery')
        # Get transformation matrices
        T_socket2fpi = self._T_socket2fpi
        T_socket2fpi_twist = self._T_socket2fpi_twist
        T_base2socket: sm.SE3 = ud.T_base2socket
        # Apply transformation chain
        T_base2fpi = T_base2socket * T_socket2fpi
        T_base2fpi_twist = T_base2socket * T_socket2fpi_twist

        # Getting in junction between plug and robot
        with self.pilot.context.motion_control():
            self.pilot.move_to_tcp_pose(T_base2fpi_twist, time_out=5.0)
            # Check if robot is in target area
            xyz_base2ft_base_est = T_base2fpi_twist.t
            xyz_base2ft_base_meas = self.pilot.robot.tcp_pos
            error = np.linalg.norm(xyz_base2ft_base_est - xyz_base2ft_base_meas)
            if error > 0.01:
                raise RuntimeError(f"Remaining position error {error} to alignment state is to large. "
                                    f"Robot is probably in an undefined condition.")
        # Start to apply some force
        with self.pilot.context.force_control():
            # Move further to apply better connection
            _ = self.pilot.tcp_force_mode(
                    wrench=np.array([0.0, 0.0, 30.0, 0.0, 0.0, 0.0]),
                    compliant_axes=[0, 0, 1, 0, 0, 0],
                    distance=0.01,  # 1cm
                    time_out=3.0)
            # Fix plug via twisting end-effector
            success = self.pilot.screw_ee_force_mode(4.0, np.pi / 2, 12.0)
            if not success:
                raise RuntimeError(f"Robot did not succeed in closing the twist lock. "
                                   f"Robot is probably in an undefined condition.")
            # Calculate error
            T_base2fpi_est = T_base2fpi
            T_base2fpi_meas = self.pilot.robot.tcp_pose
            T_fpi_meas2fpi_est = T_base2fpi_meas.inv() * T_base2fpi_est
            lin_error = np.linalg.norm(T_fpi_meas2fpi_est.t)
            ang_error = T_fpi_meas2fpi_est.angdist()
            if lin_error > 0.0075:
                raise RuntimeError(f"Remaining linear error {lin_error} is to large. "
                                   f"Robot is probably in an undefined condition.")
            if ang_error > 0.05:
                raise RuntimeError(f"Remaining angular error {ang_error} is to large."
                                   f"Robot is probably in an undefined condition.")
            rospy.loginfo(f"Arm connected to the plug with a residual error of: "
                          f"(lin={1000 * lin_error}mm; ang={np.rad2deg(ang_error)}deg)")
        return self.uc.request_action(out.ConnectToCarTwist.plug_in_bat_connect, out.Common.stop)


class RemovePlugFromBattery(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCarTwist.plug_in_bat_post_connect])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start removing the plug from battery')
        plug_out_ft = np.array([0.0, 0.0, -abs(self.cfg.data['force']), 0.0, 0.0, 0.0])
        with self.pilot.context.force_control():
            success = self.pilot.tcp_force_mode(
                wrench=plug_out_ft,
                compliant_axes=[0, 0, 1, 0, 0, 0],
                distance=self.cfg.data['moving_distance'],
                time_out=self.cfg.data['force_mode_time_out'])
        if not success:
            raise RuntimeError(f"Error while trying to unplug. Plug is probably still connected.")
        rospy.loginfo(f"Plug successfully removed from the battery socket.")
        return self.uc.request_action(out.ConnectToCarTwist.plug_in_bat_post_connect, out.Common.stop)


class MovePlugToCar(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCarTwist.plug_in_car_pre_obs])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start moving the plug to the car')
        rospy.logdebug(f"Car observation joint positions: {self.cfg.data['observation_joint_position']}")
        with self.pilot.context.position_control():
            self.pilot.move_to_joint_pos(self.cfg.data['observation_joint_position'])
        rospy.loginfo(f"Plug ended in car observation pose: "
                      f"Base-TCP = {ur_pilot.utils.se3_to_str(self.pilot.robot.tcp_pose)}")
        return self.uc.request_action(out.ConnectToCarTwist.plug_in_car_pre_obs, out.Common.stop)


class ObserveSocketOnCar(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCarTwist.plug_in_car_post_obs],
                       output_keys=['T_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start observing the socket on the car')
        # Create detector
        dtt_cfg_fp = self.cfg.data['detector_dir'].joinpath(self.cfg.data['detector_cfg'])
        dtt = pd.factory.create(dtt_cfg_fp)
        # Link camera
        if self.pilot.cam is None:
            raise RuntimeError(f"No camera in ur-pilot registered. Observation not possible.")
        dtt.register_camera(self.pilot.cam)

        # Search for ArUco pattern
        found = False
        # Use time out to exit loop
        _t_out = 5.0
        _t_start = _t_now()
        while _t_now() - _t_start <= _t_out and not found:
            # Give the robot some time to stop
            time.sleep(1.0)
            found, T_cam2socket = dtt.find_pose()
            if found:
                # Search for transformation from base to socket
                T_flange2cam = self.pilot.cam_mdl.T_flange2camera
                T_base2flange = self.pilot.get_pose('flange')
                T_base2socket = T_base2flange * T_flange2cam * T_cam2socket
        if found:
            ud.T_base2socket = T_base2socket
        else:
            raise RuntimeError(f"Can't find socket."
                               f"Make sure detector is proper set up and pattern is in camera view")
        rospy.loginfo(f"Found socket pose: Base-Socket = {ur_pilot.utils.se3_to_str(T_base2socket)}")
        return self.uc.request_action(out.ConnectToCarTwist.plug_in_car_post_obs, out.Common.stop)


class MovePlugToCarPreConnect(State):

    _T_socket2pre_connect = sm.SE3().Trans([0.0, 0.0, 0.0 - 0.02])
    
    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCarTwist.plug_in_car_pre_connect], 
                       input_keys=['T_base2socket'],
                       output_keys=['T_base2socket'])

    def execute(self, ud):
        print(), rospy.loginfo('Start moving the plug to pre-insert pose on the car')
        # Get transformation matrices
        T_base2socket = ud.T_base2socket
        T_socket2pre_connect = self._T_socket2pre_connect
        # Apply transformation chain
        T_base2pre_connect = T_base2socket * T_base2pre_connect
        # Perform actions
        with self.pilot.context.position_control():
            # Move to pre-pose to hook up to plug
            self.pilot.move_to_tcp_pose(T_base2pre_connect)
        rospy.loginfo(f"Plug ended in pre-insert pose: "
                      f"Base-TCP = {ur_pilot.utils.se3_to_str(self.pilot.robot.tcp_pose)}")
        return self.uc.request_action(out.ConnectToCarTwist.plug_in_car_pre_connect, out.Common.stop)


class InsertPlugToCar(State):

    _T_socket2fpi = sm.SE3().Trans([0.0, 0.0, 0.034])
    _T_socket2junction = sm.SE3().Trans([0.0, 0.0, 0.02])

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCarTwist.plug_in_car_connect],
                       input_keys=['T_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start inserting the plug to the car')
        # Get transformation matrices
        T_socket2fpi = self._T_socket2fpi
        T_socket2junction = self._T_socket2junction
        T_base2socket: sm.SE3 = ud.T_base2socket
        # Apply transformation chain
        T_base2fpi = T_base2socket * T_socket2fpi
        T_base2junction = T_base2socket * T_socket2junction
        # Perform actions
        with self.pilot.context.motion_control():
            self.pilot.move_to_tcp_pose(T_base2junction, time_out=4.0)
            # Check if robot is in target area
            xyz_base2jct_base_est = T_base2junction.t
            xyz_base2jct_base_meas = self.pilot.robot.tcp_pos
            error_z = np.squeeze(sm.SO3(self.pilot.robot.tcp_pose.R) * (xyz_base2jct_base_est - xyz_base2jct_base_meas))[-1]
            if abs(error_z) > 0.015:
                raise RuntimeError(f"Remaining position error {error_z} to alignment state is to large. "
                                   f"Robot is probably in an undefined condition.")
        # Start to apply some force
        with self.pilot.context.force_control():
            # Try to fully plug in
            self.pilot.plug_in_force_ramp(f_axis='z', f_start=60.0, f_end=120, duration=3.0)
            # Check if robot is in target area
            xyz_base2fpi_base_est = T_base2fpi.t
            xyz_base2fpi_base_meas = self.pilot.robot.tcp_pos
            error = np.linalg.norm(xyz_base2fpi_base_est - xyz_base2fpi_base_meas)
            if error > 0.01:
                raise RuntimeError(f"Remaining position error {error} is to large. "
                                   f"Robot is probably in an undefined condition.")
        rospy.loginfo(f"Plug inserted in car with a residual position error of {1000 * error} mm")
        return self.uc.request_action(out.ConnectToCarTwist.plug_in_car_connect, out.Common.stop)


class ReleasePlugOnCar(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCarTwist.arm_in_car_post_connect])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start releasing the arm from the plug on the car')
        with self.pilot.force_control():
            # Release plug via twisting end-effector
            success = self.pilot.screw_ee_force_mode(4.0, -np.pi / 2, 12.0)
            if not success:
                raise RuntimeError(f"Robot did not succeed in opening the twist lock. "
                                    f"Robot is probably in an undefined condition.")
            release_ft = np.array([0.0, 0.0, -25.0, 0.0, 0.0, 0.0])
            success = self.pilot.frame_force_mode(
                wrench=release_ft,
                compliant_axes=[1, 1, 1, 0, 0, 0],
                distance=0.04,
                time_out=5.0,
                frame='flange'
            )
            if not success:
                raise RuntimeError(
                    f"Error while trying to release the lock. Robot end-effector is probably still connected.")

        rospy.loginfo(f"Arm successfully released from the plug on the car.")
        return self.uc.request_action(out.ConnectToCarTwist.arm_in_car_post_connect, out.Common.stop)


class MoveArmToDrivePos(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCarTwist.arm_in_driving_pose])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start moving the arm to drive configuration')
        rospy.logdebug(f"Driving joint positions: {self.cfg.data['drive_joint_position']}")
        with self.pilot.context.position_control():
            self.pilot.move_to_joint_pos(self.cfg.data['drive_joint_position'])
        rospy.loginfo(f"Arm ended in drive pose: "
                      f"Base-TCP = {ur_pilot.utils.se3_to_str(self.pilot.robot.tcp_pose)}")
        return self.uc.request_action(out.ConnectToCarTwist.arm_in_driving_pose, out.Common.stop)
