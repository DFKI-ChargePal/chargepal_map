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


class MoveArmToCar(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.DisconnectFromCar.arm_in_car_obs])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move arm to car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        
        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.DisconnectFromCar.arm_in_car_obs
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class ObservePlugOnCar(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.DisconnectFromCar.arm_in_car_pre_connect],
                         output_keys=['xyz_xyzw_base2socket'])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Observe plug on car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.DisconnectFromCar.arm_in_car_pre_connect
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class GraspPlugOnCar(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_car_connect],
                         input_keys=['xyz_xyzw_base2socket'])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Grasp plug on car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.DisconnectFromCar.plug_in_car_connect
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class RemovePlugFromCar(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_car_post_connect])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Remove plug from car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.DisconnectFromCar.plug_in_car_post_connect
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class MovePlugToBattery(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_bat_obs])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move plug to battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.DisconnectFromCar.plug_in_bat_obs
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class ObserveSocketOnBattery(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_bat_pre_connect],
                         output_keys=['xyz_xyzw_base2socket'])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Observe socket on battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.DisconnectFromCar.plug_in_bat_pre_connect
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class InsertPlugToBattery(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_bat_connect],
                         input_keys=['xyz_xyzw_base2socket'])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Insert plug to battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.DisconnectFromCar.plug_in_bat_connect
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class ReleasePlugOnBattery(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.DisconnectFromCar.arm_in_bat_post_connect])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Release plug on battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.DisconnectFromCar.arm_in_bat_post_connect
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")


class MoveArmToDrivePos(State):

    def __init__(self, config: dict[str, Any]):
        super().__init__(outcomes=[out.Common.stop, out.DisconnectFromCar.arm_in_driving_pose])
        self.cfg = StateConfig(type(self), config=config)
        self.usr_srvs = UserServices()

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move arm to drive pos')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #

        usr_action = self.usr_srvs.wait_for_user(self.cfg.data['step_by_user'])
        if usr_action == UserServices.continue_process:
            return out.DisconnectFromCar.arm_in_driving_pose
        elif usr_action == UserServices.stop_process:
            return out.Common.stop
        else:
            raise ValueError(f"Undefined user action {usr_action}")
