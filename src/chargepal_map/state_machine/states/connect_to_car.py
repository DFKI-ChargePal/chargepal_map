from __future__ import annotations

# global
import time
import rospy
from smach import State

# local
import chargepal_map.state_machine.outcomes as out
from chargepal_map.state_machine.user_srvs import UserServices
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any

_time_out = 1.0


class MoveArmToBattery(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.ConnectToCar.arm_in_bat_obs])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move arm to battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        
        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.ConnectToCar.arm_in_bat_obs
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class ObservePlugOnBattery(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop,out.ConnectToCar.arm_in_bat_pre_connect], 
                         output_keys=['xyz_xyzw_base2socket'])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Observe plug on battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.ConnectToCar.arm_in_bat_pre_connect
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class GraspPlugOnBattery(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.ConnectToCar.plug_in_bat_connect], 
                         input_keys=['xyz_xyzw_base2socket'])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Grasp plug on battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.ConnectToCar.plug_in_bat_connect
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class RemovePlugFromBattery(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.ConnectToCar.plug_in_bat_post_connect])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Remove plug from battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.ConnectToCar.plug_in_bat_post_connect
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class MovePlugToCar(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.ConnectToCar.plug_in_car_obs])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Move plug to car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.ConnectToCar.plug_in_car_obs
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class ObserveSocketOnCar(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.ConnectToCar.plug_in_car_pre_connect],
                         output_keys=['xyz_xyzw_base2socket'])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Observe socket on car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.ConnectToCar.plug_in_car_pre_connect
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class InsertPlugToCar(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.ConnectToCar.plug_in_car_connect],
                         input_keys=['xyz_xyzw_base2socket'])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Insert plug to car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.ConnectToCar.plug_in_car_connect
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class ReleasePlugOnCar(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.ConnectToCar.arm_in_car_post_connect])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Release plug on car ')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.ConnectToCar.arm_in_car_post_connect
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class MoveArmToDrivePos(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.ConnectToCar.arm_in_driving_pose])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        print('Move arm to drive pos')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.ConnectToCar.arm_in_driving_pose
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")
