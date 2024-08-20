""" This file implements the state >>MoveToPeripheryObs<< """
from __future__ import annotations
# libs
import rospy
import ur_pilot
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


class MoveToPeripheryObs(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self,
                       outcomes=[out.periphery_pre_obs, out.job_stopped],
                       input_keys=['job', 'cart_name', 'station_name'],
                       output_keys=['job', 'cart_name', 'station_name'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        # Try to find matching configuration
        job: Job = ud.job
        cfg_data = self.cfg.extract_data(ud.station_name)
        job_data = cfg_data[job.get_id()]
        vel = cfg_data['vel']
        acc = cfg_data['acc']
        if job_data is None:
            raise KeyError(f"Can't find configuration data for the job: {job}")
        outcome = out.periphery_pre_obs
        if self.user_cb is not None:
            rospy.loginfo(f"Ready to move the arm to plug scene observation configuration")
            outcome = self.user_cb.request_action(outcome, out.job_stopped)
        if outcome != out.job_stopped:
            if job.in_progress_mode():
                rospy.loginfo(f"Start moving the arm to the periphery scene")
                with self.pilot.context.position_control():
                    self.pilot.robot.move_path_j(job_data['joint_waypoints'], vel, acc)
            elif job.in_retry_mode():
                # Move to a robot arm in a slightly different observation pose
                rospy.loginfo(f"Start moving the arm in an observation pose again with slightly different view angle.")
                with self.pilot.context.position_control():
                    j_pos_finale = job_data['joint_waypoints'][-1]
                    self.pilot.robot.movej(j_pos_finale, vel=vel, acc=acc)
                    self.pilot.set_tcp(ur_pilot.EndEffectorFrames.CAMERA)
                    current_ee_pose = self.pilot.get_pose(ur_pilot.EndEffectorFrames.CAMERA)
                    rospy.logdebug(f"Current ee pose: {ur_pilot.utils.se3_to_str(current_ee_pose)}")
                    theta = 5.0
                    if job.retry_count % 4 == 1:
                        new_ee_pose = current_ee_pose * sm.SE3().Rx(theta, unit='deg')
                    elif job.retry_count % 4 == 2:
                        new_ee_pose = current_ee_pose * sm.SE3().Ry(theta, unit='deg')
                    elif job.retry_count % 4 == 3:
                        new_ee_pose = current_ee_pose * sm.SE3().Rx(-theta, unit='deg')
                    else:  # job.retry_count % 4 == 0:
                        new_ee_pose = current_ee_pose * sm.SE3().Ry(-theta, unit='deg')
                    rospy.logdebug(f"    New ee pose: {ur_pilot.utils.se3_to_str(new_ee_pose)}")
                    self.pilot.move_to_tcp_pose(new_ee_pose)
                self.pilot.set_tcp(ur_pilot.EndEffectorFrames.FLANGE)
            else:
                raise StateMachineError(f"Job in an invalid mode. Interrupt process")
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
