from __future__ import annotations

# libs
import rospy
from chargepal_map.ui import UserSrv

# services
from chargepal_map.srv import User, UserRequest, UserResponse


class StepByUserServer:

    def __init__(self) -> None:
        # Declare user request service
        self._usr_srv = rospy.Service('user', User, self._handle_user_request)

    def _handle_user_request(self, req: UserRequest) -> UserResponse:
        res = UserResponse()
        if req.continue_process:
            res.user_action = UserSrv.continue_process
        else:
            res.user_action = UserSrv.stop_process
        return res
