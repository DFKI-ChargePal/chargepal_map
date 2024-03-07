from __future__ import annotations

# global
import rospy
from chargepal_map.srv import User

# local
from chargepal_map.ui import UserSrv


class UserClient:

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
