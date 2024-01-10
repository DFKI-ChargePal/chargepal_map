from __future__ import annotations

# global
import time
import rospy
from smach import State

# local
import chargepal_map.state_machine.outcomes as out
from chargepal_map.state_machine.states.base import BaseState

# typing
from typing import Any
from chargepal_map.state_machine.process import ProcessABC

_time_out = 1.0


class MoveArmToBattery(State, BaseState):

    def __init__(self, config: dict[str, Any], process: ProcessABC):
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCar.arm_in_bat_obs])
        BaseState.__init__(self, config, process)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move arm to battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.ConnectToCar.arm_in_bat_obs, out.Common.stop)


class ObservePlugOnBattery(State, BaseState):

    def __init__(self, config: dict[str, Any], process: ProcessABC):
        State.__init__(self, 
                       outcomes=[out.Common.stop,out.ConnectToCar.arm_in_bat_pre_connect],
                       output_keys=['xyz_xyzw_base2socket'])
        BaseState.__init__(self, config, process)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Observe plug on battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.ConnectToCar.arm_in_bat_pre_connect, out.Common.stop)


class GraspPlugOnBattery(State, BaseState):

    def __init__(self, config: dict[str, Any], process: ProcessABC):
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCar.plug_in_bat_connect],
                       input_keys=['xyz_xyzw_base2socket'])
        BaseState.__init__(self, config, process)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Grasp plug on battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.ConnectToCar.plug_in_bat_connect, out.Common.stop)


class RemovePlugFromBattery(State, BaseState):

    def __init__(self, config: dict[str, Any], process: ProcessABC):
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCar.plug_in_bat_post_connect])
        BaseState.__init__(self, config, process)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Remove plug from battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.ConnectToCar.plug_in_bat_post_connect, out.Common.stop)


class MovePlugToCar(State, BaseState):

    def __init__(self, config: dict[str, Any], process: ProcessABC):
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCar.plug_in_car_obs])
        BaseState.__init__(self, config, process)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move plug to car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.ConnectToCar.plug_in_car_obs, out.Common.stop)


class ObserveSocketOnCar(State, BaseState):

    def __init__(self, config: dict[str, Any], process: ProcessABC):
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCar.plug_in_car_pre_connect],
                       output_keys=['xyz_xyzw_base2socket'])
        BaseState.__init__(self, config, process)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Observe socket on car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.ConnectToCar.plug_in_car_pre_connect, out.Common.stop)


class InsertPlugToCar(State, BaseState):

    def __init__(self, config: dict[str, Any], process: ProcessABC):
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.ConnectToCar.plug_in_car_connect],
                       input_keys=['xyz_xyzw_base2socket'])
        BaseState.__init__(self, config, process)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Insert plug to car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.ConnectToCar.plug_in_car_connect, out.Common.stop)


class ReleasePlugOnCar(State, BaseState):

    def __init__(self, config: dict[str, Any], process: ProcessABC):
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCar.arm_in_car_post_connect])
        BaseState.__init__(self, config, process)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Release plug on car ')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.ConnectToCar.arm_in_car_post_connect, out.Common.stop)


class MoveArmToDrivePos(State, BaseState):

    def __init__(self, config: dict[str, Any], process: ProcessABC):
        State.__init__(self, outcomes=[out.Common.stop, out.ConnectToCar.arm_in_driving_pose])
        BaseState.__init__(self, config, process)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move arm to drive pos')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.ConnectToCar.arm_in_driving_pose, out.Common.stop)
