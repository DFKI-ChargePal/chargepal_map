from __future__ import annotations

import rospy
from time import perf_counter
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest

# typing
from typing import Any


class Singleton(type):
    
    _instances: dict[type, type] = {}

    def __call__(cls, *args: Any, **kwargs: Any) -> type:
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class UserRequestService:

    _block_duration = 1.0  # Block service call for x seconds

    def __init__(self, srv_name: str) -> None:
        self._t_cb_call = perf_counter()
        self._t_rq_call = self._t_cb_call
        self.ros_srv = rospy.Service(srv_name, Empty, self._callback)

    def _callback(self, req: EmptyRequest) -> EmptyRequest:
        _t_now = perf_counter()
        if _t_now - self._t_cb_call > self._block_duration:
            self._t_cb_call = _t_now
        return EmptyResponse()

    def user_request(self) -> bool:
        if self._t_rq_call != self._t_cb_call:
            self._t_rq_call = self._t_cb_call
            return True
        else:
            return False


class UserServices(metaclass=Singleton):

    continue_process = 'continue_process'
    stop_process = 'stop_process'

    def __init__(self) -> None:
        self._user_update_rate = rospy.Rate(10) # 10hz
        self._cont_proc = UserRequestService(self.continue_process)
        self._stop_proc = UserRequestService(self.stop_process)

    def wait_for_user(self) -> str:
        while True:
            self._user_update_rate.sleep()
            if self._cont_proc.user_request():
                return self.continue_process
            elif self._stop_proc.user_request():
                return self.stop_process
