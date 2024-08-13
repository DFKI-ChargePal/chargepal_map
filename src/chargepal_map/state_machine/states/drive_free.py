""" This file implements the state >>DriveFree<< """
from __future__ import annotations

# libs
import time
import rospy
from smach import State

from chargepal_map.job import Job
from chargepal_map.state_machine import outcomes as out
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.state_config import StateConfig
from chargepal_map.state_machine.utils import (
    state_header, 
    state_footer,
)
from chargepal_services.srv import (  # type: ignore
    stopFreeDriveArm, 
    stopFreeDriveArmRequest, 
    stopFreeDriveArmResponse,
)

# typing
from typing import Any
from ur_pilot import Pilot


class DriveFree(State):

    _log_rate = 10.0  # rate to output logging in seconds [sec.]

    class StopService:

        def __init__(self) -> None:
            # Declare user request service
            self._usr_srv = rospy.Service(
                'robot_arm/stop_free_drive_arm', stopFreeDriveArm, self._callback)
            self.stop = False

        def _callback(self, req: stopFreeDriveArmRequest) -> stopFreeDriveArmResponse:
            res = stopFreeDriveArmResponse()
            self.stop = True
            res.success = True
            return res
        
        def destroy(self) -> None:
            self._usr_srv.shutdown(f"Shutdown service since job is down.")

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.job_stopped, out.job_complete],
                       input_keys=['job'],
                       output_keys=['job'])

    def execute(self, ud: Any) -> str:
        print(state_header(type(self)))
        rospy.loginfo(f"Call service 'robot_arm/stop_free_drive_arm' to stop free drive mode")
        job: Job = ud.job
        # cfg_data = self.cfg.extract_data(ud.battery_id)
        usr_srv = DriveFree.StopService()
        with self.pilot.context.teach_in_control():
            _t_ref = time.perf_counter()
            while not usr_srv.stop:
                if time.perf_counter() - _t_ref > self._log_rate:
                    rospy.logdebug(f"Call service 'robot_arm/stop_free_drive_arm' to stop free drive mode")
                    _t_ref = time.perf_counter()
        rospy.loginfo(f"Free drive mode stopped.")
        usr_srv.destroy()
        outcome = out.job_complete
        job.track_state(type(self))
        print(state_footer(type(self)))
        return outcome
