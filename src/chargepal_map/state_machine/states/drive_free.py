""" This file implements the state >>DriveFree<< """
from __future__ import annotations

# libs
import time
import rospy
from smach import State

from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.outcomes import Outcomes as out
from chargepal_map.state_machine.state_config import StateConfig

from chargepal_services.srv import stopFreeDriveArm, stopFreeDriveArmRequest, stopFreeDriveArmResponse

# typing
from typing import Any
from ur_pilot import Pilot


class DriveFree(State):

    _log_rate = 10.0  # [sec.]

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

    def __init__(self, config: dict[str, Any], pilot: Pilot, user_cb: StepByUser | None = None):
        self.pilot = pilot
        self.user_cb = user_cb
        self.cfg = StateConfig(type(self), config=config)
        State.__init__(self, 
                       outcomes=[out.stop, out.completed],
                       input_keys=['job_id'],
                       output_keys=['job_id'])

    def execute(self, ud: Any) -> str:
        print(), rospy.loginfo(f"Start free drive mode."
                               f"Call service 'robot_arm/stop_free_drive_arm' to stop free drive mode")
        usr_srv = DriveFree.StopService()
        with self.pilot.context.teach_in_control():
            _t_ref = time.perf_counter()
            while not usr_srv.stop():
                if time.perf_counter() - _t_ref > self._log_rate:
                    rospy.logdebug(f"Call service 'robot_arm/stop_free_drive_arm' to stop free drive mode")
                    _t_ref = time.perf_counter()
        rospy.loginfo(f"Free drive mode stopped.")
        outcome = out.completed
        if self.user_cb is not None:
            outcome = self.user_cb.request_action(out.completed, out.stop)
        return outcome
