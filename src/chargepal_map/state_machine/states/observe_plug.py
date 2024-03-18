""" This file implements the state >>ObservePlug<< """
from __future__ import annotations

# libs
import time
import rospy
import ur_pilot
import cvpd as pd
from smach import State
from time import perf_counter as _t_now

from chargepal_map.ui.user_client import UserClient
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any
from ur_pilot import Pilot


class ObservePlug(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.stop, out.plug_obs], output_keys=['T_base2socket'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo('Start observing the socket with plug on car')
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
        return self.uc.request_action(out.plug_obs, out.stop)
