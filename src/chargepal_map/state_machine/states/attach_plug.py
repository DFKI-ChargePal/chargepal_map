""" This file implements the state >>AttachPlug<< """
from __future__ import annotations
# libs
import time
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


class AttachPlug(State):

    _T_socket2fpi = sm.SE3().Trans([0.0, 0.0, 0.034])
    _T_socket2fpi_twist = sm.SE3().Rt(R=sm.SO3.EulerVec((0.0, 0.0, -np.pi/2)), t=(0.0, 0.0, 0.034))

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self,
                       outcomes=[out.stop, out.plug_attached],
                       input_keys=['job_id'],
                       output_keys=['job_id'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start grasping the plug on battery')
        # Get transformation matrices
        T_socket2fpi = self._T_socket2fpi
        T_socket2fpi_twist = self._T_socket2fpi_twist
        T_base2socket = sm.SE3().Rt(
            R=sm.SO3.EulerVec(self.cfg.data['eulvec_base2socket']), 
            t=self.cfg.data['trans_base2socket']
            )
        # Apply transformation chain
        T_base2fpi = T_base2socket * T_socket2fpi
        T_base2fpi_twist = T_base2socket * T_socket2fpi_twist

        # Start to apply some force
        with self.pilot.context.force_control():
            # Move further to apply better connection
            _ = self.pilot.tcp_force_mode(
                    wrench=np.array([0.0, 0.0, 30.0, 0.0, 0.0, 0.0]),
                    compliant_axes=[0, 0, 1, 0, 0, 0],
                    distance=0.02,  # 2cm
                    time_out=3.0)
            time.sleep(0.5)
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
            ang_error = T_base2fpi_est.angdist(T_base2fpi_meas)
            if lin_error > 0.0075:
                raise RuntimeError(f"Remaining linear error {lin_error} is to large. "
                                   f"Robot is probably in an undefined condition.")
            if ang_error > 0.05:
                raise RuntimeError(f"Remaining angular error {ang_error} is to large."
                                   f"Robot is probably in an undefined condition.")
            rospy.loginfo(f"Arm connected to the plug with a residual error of: "
                          f"(lin={1000 * lin_error}mm; ang={np.rad2deg(ang_error)}deg)")
        return self.uc.request_action(out.plug_attached, out.stop)
