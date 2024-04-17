""" This file implements the state >>ObservePlug<< """
from __future__ import annotations

# libs
import rospy
import ur_pilot
from smach import State
import spatialmath as sm

from chargepal_map.core import job_ids
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.utils import StateMachineError

# typing
from typing import Any
from ur_pilot import Pilot


class ObservePlug(State):

    _T_marker2obs_close = sm.SE3().Rt(R=sm.SO3.EulerVec((0.0, 0.15, 0.0)), t=(0.1, -0.025, -0.1))

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.stop, out.plug_obs], 
                       input_keys=['job_id'], 
                       output_keys=['job_id', 'T_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start observing the socket with plug')
        job_id = ud.job_id
        # Get plug type
        if job_id in job_ids.type2_female():
            plug_type = 'type2_female'
        elif job_id in job_ids.type2_male():
            plug_type = 'type2_male'
        elif job_id in job_ids.ccs_female():
            plug_type = 'ccs_female'
        else:
            raise StateMachineError(f"Invalid or undefined job ID '{job_id}' for this state.")
        with self.pilot.plug_model.context(plug_type):
            if self.cfg.data[job_id]['two_step_approach']:
                dtt_cfg_fp = self.cfg.data['detector'][self.cfg.data[job_id]['detector_i']]
                for _ in range(2):
                    found, T_base2marker = self.pilot.find_target_pose(
                        detector_fp=dtt_cfg_fp,
                        time_out=self.cfg.data[job_id]['time_out'])
                    if found:
                        with self.pilot.context.position_control():
                            self.pilot.set_tcp(ur_pilot.EndEffectorFrames.PLUG_SAFETY)
                            T_base2obs_close = T_base2marker * self._T_marker2obs_close
                            self.pilot.move_to_tcp_pose(T_base2obs_close)
            dtt_cfg_fp = self.cfg.data['detector'][self.cfg.data[job_id]['detector_ii']]
            found, T_base2socket = self.pilot.find_target_pose(
                detector_fp=dtt_cfg_fp,
                time_out=self.cfg.data[job_id]['time_out'])
        rospy.loginfo(f"Finding socket pose successfully: {found}")
        if found:
            rospy.logdebug(f"Transformation: Base-Socket = {ur_pilot.utils.se3_to_str(T_base2socket)}")
            ud.T_base2socket = T_base2socket
        else:
            raise StateMachineError(f"Can't find socket. "
                                    f"Make sure detector is proper set up and pattern is in camera view")
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.plug_obs, out.stop)
        return outcome
