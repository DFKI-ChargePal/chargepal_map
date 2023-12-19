from __future__ import annotations

# global
import time
import cvpd as pd
import numpy as np
import camera_kit as ck
from smach import State
from pathlib import Path
from ur_pilot import Pilot
from time import perf_counter as _t_now
from rigmopy import Pose, Vector6d

# local
import chargepal_map.processes.outcomes as out

# typing
from typing import Any


_time_out = 1.0


"""   move arm to battery   """
class MoveArmToBattery(State):

    _battery_obs_j_pos = (3.773, -1.159, 1.914, -0.263, 2.073, -1.267)

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_bat_obs])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos(self._battery_obs_j_pos)
        time.sleep(_time_out)
        return out.ConnectToCar.arm_in_bat_obs


class ImplicateMoveArmToBattery(State):

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_bat_obs])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.773, -1.159, 1.914, -0.263, 2.073, -1.267])
        time.sleep(_time_out)
        return out.ConnectToCar.arm_in_bat_obs


"""   observe plug on battery   """
class ObservePlugOnBattery(State):
    
    _cam_config_fp = Path("/home/gejo02/chargepal_ws/src/chargepal/chargepal_map/config/camera_info/realsense_tcp_cam/calibration/coefficients.toml")
    _dtt_config_fp = Path("/home/gejo02/chargepal_ws/src/chargepal/chargepal_map/config/detector/charuco_ads_socket_ty2_adj.yaml")

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_bat_pre_connect], output_keys=['t_base2socket'])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        cam = ck.create('realsense_tcp_cam')
        cam.load_coefficients(self._cam_config_fp)
        dtt = pd.ArucoPatternDetector(self._dtt_config_fp)
        dtt.register_camera(cam)
        self._pilot.robot.register_ee_cam(cam)

        # Search for ArUco pattern
        found = False
        # Use time out to exit loop
        _t_out = 5.0
        _t_start = _t_now()
        while _t_now() - _t_start <= _t_out and not found:

            found, pose_cam2socket = dtt.find_pose()
            if found:
                # Search for transformation from base to socket
                # Get transformation matrices
                T_plug2cam = self._pilot.robot.cam_mdl.T_flange2camera
                T_base2plug = self._pilot.robot.get_tcp_pose().transformation
                T_cam2socket = Pose().from_xyz_xyzw(*pose_cam2socket).transformation
                # Apply transformation chain
                T_base2socket = T_base2plug @ T_plug2cam @ T_cam2socket
            # Give the robot some time to stop
            time.sleep(1.0)

        if found:
            ud.t_base2socket = T_base2socket
        else:
            raise RuntimeError(f"Can't find socket."
                                f"Make sure detector is proper set up and pattern is in camera view")

        return out.ConnectToCar.arm_in_bat_pre_connect


class ImplicateObservePlugOnBattery(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_bat_pre_connect])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.670, -0.987, 1.704, -0.154, 1.980, -1.267])
        time.sleep(_time_out)
        return out.ConnectToCar.arm_in_bat_pre_connect


"""   grasp plug on battery   """
class GraspPlugOnBattery(State):

    _pose_socket2hook = Pose().from_xyz([0.0-0.01, 0.0, 0.034])
    _pose_socket2hook_pre = Pose().from_xyz([0.03, 0.0, 0.034 - 0.02])
    _pose_socket2hook_itm = Pose().from_xyz([0.03, 0.0, 0.034])

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_bat_connect], input_keys=['t_base2socket'])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        # Get transformation matrices
        T_base2socket = ud.t_base2socket
        T_socket2hook = self._pose_socket2hook.transformation
        T_socket2hook_pre = self._pose_socket2hook_pre.transformation
        T_socket2hook_itm = self._pose_socket2hook_itm.transformation
        # Apply transformation chain
        T_base2hook = T_base2socket @ T_socket2hook
        T_base2hook_pre = T_base2socket @ T_socket2hook_pre
        T_base2hook_itm = T_base2socket @ T_socket2hook_itm
        # Perform actions
        with self._pilot.position_control():
            # Move to pre-pose to hook up to plug
            self._pilot.move_to_tcp_pose(T_base2hook_pre.pose)
        with self._pilot.motion_control():
            self._pilot.move_to_tcp_pose(T_base2hook_itm.pose, time_out=5.0)
            self._pilot.move_to_tcp_pose(T_base2hook.pose, time_out=5.0)
        return out.ConnectToCar.plug_in_bat_connect


class ImplicateGraspPlugOnBattery(State):

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_bat_connect])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.634, -0.917, 1.634, -0.152, 1.977, -1.267])
        time.sleep(_time_out)
        return out.ConnectToCar.plug_in_bat_connect
    

"""   remove plug from battery   """
class RemovePlugFromBattery(State):
    
    _plug_out_ft = Vector6d().from_xyzXYZ([0.0, 0.0, -150.0, 0.0, 0.0, 0.0])

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_bat_post_connect])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        with self._pilot.force_control():
            success = self._pilot.tcp_force_mode(
                wrench=self._plug_out_ft,
                compliant_axes=[0, 0, 1, 0, 0, 0],
                distance=0.05,
                time_out=10.0)
        if not success:
            raise RuntimeError(f"Error while trying to unplug. Plug is probably still connected.")
        return out.ConnectToCar.plug_in_bat_post_connect


class ImplicateRemovePlugFromBattery(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_bat_post_connect])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.663, -1.045, 1.736, -0.127, 1.978, -1.267])
        time.sleep(_time_out)
        return out.ConnectToCar.plug_in_bat_post_connect


"""   move plug to car   """
class MovePlugToCar(State):
    
    _car_obs_j_pos = (3.476, -1.500, 1.756, 0.050, 1.887, -1.267)

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_obs])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos(self._car_obs_j_pos)
        time.sleep(_time_out)
        return out.ConnectToCar.plug_in_car_obs
    

class ImplicateMovePlugToCar(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_obs])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.476, -1.500, 1.756, 0.050, 1.887, -1.267])
        time.sleep(_time_out)
        return out.ConnectToCar.plug_in_car_obs


"""   observe socket on car   """
class ObserveSocketOnCar(State):
    
    _cam_config_fp = Path()
    _dtt_config_fp = Path()

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_pre_connect], output_keys=['t_base2socket'])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        cam = ck.create('realsense_tcp_cam')
        cam.load_coefficients(self._cam_config_fp)
        dtt = pd.CharucoDetector(self._dtt_config_fp)
        dtt.register_camera(cam)
        self._pilot.robot.register_ee_cam(cam)

        # Search for ArUco pattern
        found = False
        # Use time out to exit loop
        _t_out = 5.0
        _t_start = _t_now()
        while _t_now() - _t_start <= _t_out and not found:

            found, pose_cam2socket = dtt.find_pose()
            if found:
                # Search for transformation from base to socket
                # Get transformation matrices
                T_plug2cam = self._pilot.robot.cam_mdl.T_flange2camera
                T_base2plug = self._pilot.robot.get_tcp_pose().transformation
                T_cam2socket = Pose().from_xyz_xyzw(*pose_cam2socket).transformation
                # Apply transformation chain
                T_base2socket = T_base2plug @ T_plug2cam @ T_cam2socket
            # Give the robot some time to stop
            time.sleep(1.0)

        if found:
            ud.t_base2socket = T_base2socket
        else:
            raise RuntimeError(f"Can't find socket."
                                f"Make sure detector is proper set up and pattern is in camera view")

        return out.ConnectToCar.plug_in_car_pre_connect


class ImplicateObserveSocketOnCar(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_pre_connect])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.346, -1.467, 1.835, -0.363, 1.766, -1.578])
        time.sleep(_time_out)
        return out.ConnectToCar.plug_in_car_pre_connect


"""   insert plug to car   """
class InsertPlugToCar(State):

    _pose_socket2socket_pre = Pose().from_xyz(xyz=[0.0, 0.0, 0.0 - 0.02])
    _pose_socket2fpi = Pose().from_xyz(xyz=[0.0, 0.0, 0.034])
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_connect], input_keys=['t_base2socket'])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        # Get transformation matrices
        T_base2socket = ud.t_base2socket
        T_socket2fpi = self._pose_socket2fpi.transformation
        T_socket2socket_pre = self._pose_socket2socket_pre.transformation
        # Apply transformation chain
        T_base2socket_pre = T_base2socket @ T_socket2socket_pre
        # Perform actions
        with self._pilot.position_control():    
                # Move to socket with some safety distance
                self._pilot.move_to_tcp_pose(T_base2socket_pre.pose)
        time.sleep(1.0)
        with self._pilot.force_control():
            self._pilot.one_axis_tcp_force_mode(axis='z', force=20.0, time_out=4.0)
            self._pilot.plug_in_force_ramp(f_axis='z', f_start=75.0, f_end=125, duration=4.0)
            self._pilot.relax(2.0)
            # Check if robot is in target area
            T_base2fpi = T_base2socket @ T_socket2fpi
            xyz_base2fpi_base_est = np.reshape(T_base2fpi.tau, 3)
            xyz_base2fpi_base_meas = np.reshape(self._pilot.robot.get_tcp_pose().xyz, 3)
            error = np.sqrt(np.sum(np.square(xyz_base2fpi_base_est - xyz_base2fpi_base_meas)))
            if error > 0.01:
                raise RuntimeError(f"Remaining position error {error} is to large. "
                                   f"Robot is probably in an undefined condition.")
        return out.ConnectToCar.plug_in_car_connect


class ImplicateInsertPlugToCar(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_connect])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.337, -1.412, 1.761, -0.362, 1.766, -1.578])
        time.sleep(_time_out)
        return out.ConnectToCar.plug_in_car_connect


"""   release plug on car   """
class ReleasePlugOnCar(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_car_post_connect])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        with self._pilot.motion_control():
            # Move 30 mm in tcp x direction to open the plug lock
            pose_tcp2target = Pose().from_xyz([0.030, 0.0, 0.0]) 
            pose_base2tcp = self._pilot.robot.get_tcp_pose()
            pose_base2target = pose_base2tcp * pose_tcp2target
            self._pilot.move_to_tcp_pose(pose_base2target, time_out=5.0)
            # Move -20 mm in tcp z direction to disconnect from plug
            pose_tcp2target = Pose().from_xyz([0.0, 0.0, -0.020])
            pose_base2tcp = self._pilot.robot.get_tcp_pose()
            pose_base2target = pose_base2tcp * pose_tcp2target
            self._pilot.move_to_tcp_pose(pose_base2target, time_out=3.0)

        # Check if robot is in target area
        xyz_base2target_base_est = np.reshape(pose_base2target.xyz, 3)
        xyz_base2target_base_meas = np.reshape(self._pilot.robot.get_tcp_pose().xyz, 3)
        error = np.sqrt(np.sum(np.square(xyz_base2target_base_est - xyz_base2target_base_meas)))
        if error > 0.01:
            raise RuntimeError(f"Remaining position error {error} is to large. "
                                f"Robot is probably in an undefined condition.")
        return out.ConnectToCar.arm_in_car_post_connect


class ImplicateReleasePlugOnCar(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_car_post_connect])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.334, -1.451, 1.859, -0.422, 1.770, -1.578])
        time.sleep(_time_out)
        return out.ConnectToCar.arm_in_car_post_connect


"""   move arm to drive pos   """
class MoveArmToDrivePos(State):

    _drive_j_pos = (3.431, -1.371, 2.129, -0.743, 1.869, -1.562)

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_driving_pose])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos(self._drive_j_pos)
        time.sleep(_time_out)
        return out.ConnectToCar.arm_in_driving_pose


class ImplicateMoveArmToDrivePos(State):

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_driving_pose])
        self._pilot = pilot

    def execute(self, ud: Any) -> str:
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.431, -1.371, 2.129, -0.743, 1.869, -1.562])
        time.sleep(_time_out)
        return out.ConnectToCar.arm_in_driving_pose
