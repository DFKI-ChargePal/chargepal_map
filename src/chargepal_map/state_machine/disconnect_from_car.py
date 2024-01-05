from __future__ import annotations

# global
import time
from smach import State
from pathlib import Path
# local
import state_machine.outcomes as out

# typing
from typing import Any

_time_out = 1.0


class MoveArmToCar(State):

    def __init__(self, cfg_dir: Path):
        super().__init__(outcomes=[out.DisconnectFromCar.arm_in_car_obs])
        self.cfg_dir = cfg_dir

    def execute(self, ud: Any) -> str:
        print('move arm to car')
        time.sleep(_time_out)
        return out.DisconnectFromCar.arm_in_car_obs


class ObservePlugOnCar(State):

    def __init__(self, cfg_dir: Path):
        super().__init__(outcomes=[out.DisconnectFromCar.arm_in_car_pre_connect], output_keys=['xyz_xyzw_base2socket'])
        self.cfg_dir = cfg_dir

    def execute(self, ud: Any) -> str:
        print('observe plug on car')
        time.sleep(_time_out)
        return out.DisconnectFromCar.arm_in_car_pre_connect


class GraspPlugOnCar(State):

    def __init__(self, cfg_dir: Path):
        super().__init__(outcomes=[out.DisconnectFromCar.plug_in_car_connect], input_keys=['xyz_xyzw_base2socket'])
        self.cfg_dir = cfg_dir

    def execute(self, ud: Any) -> str:
        print('grasp plug on car')
        time.sleep(_time_out)
        return out.DisconnectFromCar.plug_in_car_connect


class RemovePlugFromCar(State):

    def __init__(self, cfg_dir: Path):
        super().__init__(outcomes=[out.DisconnectFromCar.plug_in_car_post_connect])
        self.cfg_dir = cfg_dir

    def execute(self, ud: Any) -> str:
        print('remove plug from car')
        time.sleep(_time_out)
        return out.DisconnectFromCar.plug_in_car_post_connect


class MovePlugToBattery(State):

    def __init__(self, cfg_dir: Path):
        super().__init__(outcomes=[out.DisconnectFromCar.plug_in_bat_obs])
        self.cfg_dir = cfg_dir

    def execute(self, ud: Any) -> str:
        print('move plug to battery')
        time.sleep(_time_out)
        return out.DisconnectFromCar.plug_in_bat_obs


class ObserveSocketOnBattery(State):

    def __init__(self, cfg_dir: Path):
        super().__init__(outcomes=[out.DisconnectFromCar.plug_in_bat_pre_connect], output_keys=['xyz_xyzw_base2socket'])
        self.cfg_dir = cfg_dir

    def execute(self, ud: Any) -> str:
        print('observe socket on battery')
        time.sleep(_time_out)
        return out.DisconnectFromCar.plug_in_bat_pre_connect


class InsertPlugToBattery(State):

    def __init__(self, cfg_dir: Path):
        super().__init__(outcomes=[out.DisconnectFromCar.plug_in_bat_connect], input_keys=['xyz_xyzw_base2socket'])
        self.cfg_dir = cfg_dir

    def execute(self, ud: Any) -> str:
        print('insert plug to battery')
        time.sleep(_time_out)
        return out.DisconnectFromCar.plug_in_bat_connect


class ReleasePlugOnBattery(State):

    def __init__(self, cfg_dir: Path):
        super().__init__(outcomes=[out.DisconnectFromCar.arm_in_bat_post_connect])
        self.cfg_dir = cfg_dir

    def execute(self, ud: Any) -> str:
        print('release plug on battery')
        time.sleep(_time_out)
        return out.DisconnectFromCar.arm_in_bat_post_connect


class MoveArmToDrivePos(State):

    def __init__(self, cfg_dir: Path):
        super().__init__(outcomes=[out.DisconnectFromCar.arm_in_driving_pose])
        self.cfg_dir = cfg_dir

    def execute(self, ud: Any) -> str:
        print('move arm to drive pos')
        time.sleep(_time_out)
        return out.DisconnectFromCar.arm_in_driving_pose
