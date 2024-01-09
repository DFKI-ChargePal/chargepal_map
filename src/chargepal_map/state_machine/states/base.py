from __future__ import annotations

# local
from chargepal_map.state_machine.user_srvs import UserServices
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any
from actionlib import SimpleActionServer


class BaseState:

    def __init__(self, config: dict[str, Any], action_srv: SimpleActionServer):
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()
        self.action_server = action_srv
    

    def _user_request(self, continue_res: str, stop_res: str) -> str:
        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            res = continue_res
        elif usr_action == UserServices.stop_process:
            res = stop_res
        else:
            raise ValueError(f"Undefined user action {usr_action}")
        return res
