from __future__ import annotations

# local
from chargepal_map.state_machine.user_srvs import UserServices
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any
from chargepal_map.state_machine.process import ProcessABC


class BaseState:

    def __init__(self, config: dict[str, Any], process: ProcessABC):
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()
        self.process = process

    def step_by_user(self) -> bool:
        retval: bool = self.cfg.data['step_by_user']
        return retval

    def _user_request(self, continue_res: str, stop_res: str) -> str:
        if self.step_by_user():
            self.process.wait_for_usr_feedback()
        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            res = continue_res
        elif usr_action == UserServices.stop_process:
            res = stop_res
        else:
            raise ValueError(f"Undefined user action {usr_action}")
        return res
