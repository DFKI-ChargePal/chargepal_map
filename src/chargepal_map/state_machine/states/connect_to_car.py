from __future__ import annotations

# global
import time
import rospy
from smach import State

# local
import chargepal_map.state_machine.outcomes as out
from chargepal_map.ui.user_client import UserClient
from chargepal_map.state_machine.state_config import StateConfig

# typing
from typing import Any
from ur_pilot import Pilot


_time_out = 1.0


class MoveArmToBattery(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCar.arm_in_bat_obs])
        
    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move arm to battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self.uc.request_action(out.ConnectToCar.arm_in_bat_obs, out.Common.stop)


class ObservePlugOnBattery(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop,out.ConnectToCar.arm_in_bat_pre_connect],
                       output_keys=['xyz_xyzw_base2socket'])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Observe plug on battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self.uc.request_action(out.ConnectToCar.arm_in_bat_pre_connect, out.Common.stop)


class GraspPlugOnBattery(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCar.plug_in_bat_connect],
                       input_keys=['xyz_xyzw_base2socket'])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Grasp plug on battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self.uc.request_action(out.ConnectToCar.plug_in_bat_connect, out.Common.stop)


class RemovePlugFromBattery(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCar.plug_in_bat_post_connect])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Remove plug from battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self.uc.request_action(out.ConnectToCar.plug_in_bat_post_connect, out.Common.stop)


class MovePlugToCar(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCar.plug_in_car_obs])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move plug to car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self.uc.request_action(out.ConnectToCar.plug_in_car_obs, out.Common.stop)


class ObserveSocketOnCar(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCar.plug_in_car_pre_connect],
                       output_keys=['xyz_xyzw_base2socket'])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Observe socket on car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self.uc.request_action(out.ConnectToCar.plug_in_car_pre_connect, out.Common.stop)


class InsertPlugToCar(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCar.plug_in_car_connect],
                       input_keys=['xyz_xyzw_base2socket'])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Insert plug to car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self.uc.request_action(out.ConnectToCar.plug_in_car_connect, out.Common.stop)


class ReleasePlugOnCar(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCar.arm_in_car_post_connect])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Release plug on car ')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self.uc.request_action(out.ConnectToCar.arm_in_car_post_connect, out.Common.stop)


class MoveArmToDrivePos(State):

    def __init__(self, config: dict[str, Any], pilot: Pilot):
        self.pilot = pilot
        self.cfg = StateConfig(type(self), config=config)
        self.uc = UserClient(self.cfg.data['step_by_user'])
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCar.arm_in_driving_pose])

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move arm to drive pos')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self.uc.request_action(out.ConnectToCar.arm_in_driving_pose, out.Common.stop)


def default_config() -> str:

    res = ''
    states = [
        MoveArmToBattery,
        ObservePlugOnBattery,
        GraspPlugOnBattery,
        RemovePlugFromBattery,
        MovePlugToCar,
        ObserveSocketOnCar,
        InsertPlugToCar,
        ReleasePlugOnCar,
        MoveArmToDrivePos
    ]

    for s in states:
        
        res += StateConfig(s, {}).dump()

    return res


if __name__ == '__main__':
    print(default_config())
