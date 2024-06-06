from __future__ import annotations
# libs
import rospy

# services
from chargepal_map.srv import User, UserRequest, UserResponse


class UserSrv:
    continue_process = 'continue_process'
    stop_process = 'stop_process'


class StepByUser:

    def __init__(self, enable: bool) -> None:
        # Declare user request service
        self.enable = enable
        self._usr_srv = rospy.Service('user', User, self._srv_handler)
        self.continue_process = False

    def request_action(self, continue_res: str, stop_res: str) -> str:
        if self.enable:
            self.user_action = False
            rospy.loginfo(f"Waiting for user action to continue or stop the state machine.")
            while not self.user_action:
                rospy.sleep(0.1)
        else:
            self.continue_process = True
        if self.continue_process:
            res = continue_res
            rospy.loginfo(f"Continue state machine with going to next state: {res}")
        else:
            res = stop_res
            rospy.loginfo(f"Stopping state machine with going to final state: {res}")
        return res

    def _srv_handler(self, req: UserRequest) -> UserResponse:
        res = UserResponse()
        if req.continue_process:
            self.continue_process = True
        else:
            self.continue_process = False
        self.user_action = True
        return res


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


class StepByUserClient:

    def __init__(self, enable: bool):
        self.step_by_user = enable
        if self.step_by_user:
            rospy.wait_for_service('user')

    def request_action(self, continue_res: str, stop_res: str) -> str:
        if self.step_by_user:
            wait_for_usr = rospy.ServiceProxy('user', User)
            usr_action = wait_for_usr().user_action
        else:
            usr_action = UserSrv.continue_process

        if usr_action == UserSrv.continue_process:
            res = continue_res
        elif usr_action == UserSrv.stop_process:
            res = stop_res
        else:
            raise ValueError(f"Undefined user action {usr_action}")
        return res
