from __future__ import annotations

# global
import time
import rospy
from smach import State
from pathlib import Path

# local
import chargepal_map.state_machine.outcomes as out
from chargepal_map.state_machine.user_srvs import UserServices
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any

_time_out = 1.0


class MoveArmToBattery(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_bat_obs])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Move arm to battery')
        print('    Wait for user...')
        print(self.usr_srvs.wait_for_user())
        return out.ConnectToCar.arm_in_bat_obs


class ObservePlugOnBattery(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_bat_pre_connect], output_keys=['xyz_xyzw_base2socket'])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Observe plug on battery')
        time.sleep(_time_out)
        print('    Wait for user...')
        print(self.usr_srvs.wait_for_user())
        return out.ConnectToCar.arm_in_bat_pre_connect


class GraspPlugOnBattery(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_bat_connect], input_keys=['xyz_xyzw_base2socket'])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Grasp plug on battery')
        time.sleep(_time_out)
        print('    Wait for user...')
        print(self.usr_srvs.wait_for_user())
        return out.ConnectToCar.plug_in_bat_connect


class RemovePlugFromBattery(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_bat_post_connect])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Remove plug from battery')
        time.sleep(_time_out)
        print('    Wait for user...')
        self.usr_srvs.wait_for_user()
        return out.ConnectToCar.plug_in_bat_post_connect


class MovePlugToCar(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_obs])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Move plug to car')
        time.sleep(_time_out)
        print('    Wait for user...')
        self.usr_srvs.wait_for_user()
        return out.ConnectToCar.plug_in_car_obs


class ObserveSocketOnCar(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_pre_connect], output_keys=['xyz_xyzw_base2socket'])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Observe socket on car')
        time.sleep(_time_out)
        print('    Wait for user...')
        print(self.usr_srvs.wait_for_user())
        return out.ConnectToCar.plug_in_car_pre_connect


class InsertPlugToCar(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_connect], input_keys=['xyz_xyzw_base2socket'])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Insert plug to car')
        time.sleep(_time_out)
        print('    Wait for user...')
        print(self.usr_srvs.wait_for_user())
        return out.ConnectToCar.plug_in_car_connect


class ReleasePlugOnCar(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_car_post_connect])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Release plug on car ')
        time.sleep(_time_out)
        print('    Wait for user...')
        print(self.usr_srvs.wait_for_user())
        return out.ConnectToCar.arm_in_car_post_connect


class MoveArmToDrivePos(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_driving_pose])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Move arm to drive pos')
        time.sleep(_time_out)
        print('    Wait for user...')
        print(self.usr_srvs.wait_for_user())
        return out.ConnectToCar.arm_in_driving_pose
