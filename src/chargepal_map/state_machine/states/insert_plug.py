""" This file implements the state >>InsertPlug<< """
from __future__ import annotations
# libs
import rospy
import numpy as np
from smach import State
import spatialmath as sm

from chargepal_map.ui.user_client import UserClient
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any
from ur_pilot import Pilot


class InsertPlug(State):

    _T_socket2fpi = sm.SE3().Trans([0.0, 0.0, 0.034])
    _T_socket2junction = sm.SE3().Trans([0.0, 0.0, 0.02])

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.stop, out.plug_connected],
                       input_keys=['T_base2socket'],
                       output_keys=['T_base2socket'])

    def execute(self, ud) -> str:
        print(), rospy.loginfo('Start inserting the plug to the car')
        # Get transformation matrices
        T_base2socket = sm.SE3(ud.T_base2socket)
        T_socket2fpi = self._T_socket2fpi
        T_socket2junction = self._T_socket2junction
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
            self.pilot.plug_in_force_ramp(f_axis='z', f_start=30.0, f_end=60, duration=2.0)
            # Check if robot is in target area
            xyz_base2fpi_base_est = T_base2fpi.t
            xyz_base2fpi_base_meas = self.pilot.robot.tcp_pos
            error = np.linalg.norm(xyz_base2fpi_base_est - xyz_base2fpi_base_meas)
            if error > 0.01:
                raise RuntimeError(f"Remaining position error {error} is to large. "
                                   f"Robot is probably in an undefined condition.")
        rospy.loginfo(f"Plug inserted in car with a residual position error of {1000 * error} mm")
        return self.uc.request_action(out.plug_connected, out.Common.stop)
