""" This file implements the state >>MoveToPlugObs<< """
from __future__ import annotations
# libs
import rospy
import ur_pilot
import numpy as np
from smach import State
import spatialmath as sm

from chargepal_map.job import Job
from chargepal_map.state_machine import outcomes as out
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.utils import (
    state_header,
    state_footer,
    StateMachineError,
)

# typing
from typing import Any
from ur_pilot import Pilot


class MoveToPlugObs(State):

    _T_socket_save2camera = sm.SE3().Rt(R=sm.SO3.EulerVec((0.0, 0.0, -np.pi/2)), t=(-0.1, 0.0, -0.25)) * sm.SE3().Rx(-10.0, unit='deg')

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self,
                       outcomes=[out.plug_pre_obs, out.job_stopped],
                       input_keys=['job'],
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Try to find matching configuration
        job: Job = ud.job
        vel = self.cfg.data['vel']
        acc = self.cfg.data['acc']
        # Try to move the arm in front of the plug observation pattern
        outcome = out.plug_pre_obs
        if self.user_cb is not None:
            rospy.loginfo(f"Ready to move the arm in front of the plug observation pattern")
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        if outcome != out.job_stopped:
            if job.is_part_of_plug_out():
                T_base2socket_scene = job.exterior_socket.T_base2socket_scene
                if T_base2socket_scene is None:
                    raise StateMachineError(f"Missing observation of the plug scene. Interrupt process")
                # Start moving the arm
                with self.pilot.plug_model.context(plug_type=job.get_plug_type()):
                    self.pilot.set_tcp(ur_pilot.EndEffectorFrames.CAMERA)
                    T_base2camera = T_base2socket_scene * self._T_socket_save2camera
                    self.pilot.robot.movel(T_base2camera, vel, acc)
            else:
                raise StateMachineError(f"Invalid or undefined job '{job}' for this state")
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
