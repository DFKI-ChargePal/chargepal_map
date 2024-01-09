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
from actionlib import SimpleActionServer

_time_out = 1.0


class MoveArmToCar(State, BaseState):

    def __init__(self, config: dict[str, Any], action_srv: SimpleActionServer):
        State.__init__(self, outcomes=[out.Common.stop, out.DisconnectFromCar.arm_in_car_obs])
        BaseState.__init__(self, config, action_srv)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move arm to car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.DisconnectFromCar.arm_in_car_obs, out.Common.stop)


class ObservePlugOnCar(State, BaseState):

    def __init__(self, config: dict[str, Any], action_srv: SimpleActionServer):
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.DisconnectFromCar.arm_in_car_pre_connect],
                       output_keys=['xyz_xyzw_base2socket'])
        BaseState.__init__(self, config, action_srv)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Observe plug on car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.DisconnectFromCar.arm_in_car_pre_connect, out.Common.stop)


class GraspPlugOnCar(State, BaseState):

    def __init__(self, config: dict[str, Any], action_srv: SimpleActionServer):
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_car_connect],
                       input_keys=['xyz_xyzw_base2socket'])
        BaseState.__init__(self, config, action_srv)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Grasp plug on car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.DisconnectFromCar.plug_in_car_connect, out.Common.stop)


class RemovePlugFromCar(State, BaseState):

    def __init__(self, config: dict[str, Any], action_srv: SimpleActionServer):
        State.__init__(self, outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_car_post_connect])
        BaseState.__init__(self, config, action_srv)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Remove plug from car')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.DisconnectFromCar.plug_in_car_post_connect, out.Common.stop)


class MovePlugToBattery(State, BaseState):

    def __init__(self, config: dict[str, Any], action_srv: SimpleActionServer):
        State.__init__(self, outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_bat_obs])
        BaseState.__init__(self, config, action_srv)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move plug to battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.DisconnectFromCar.plug_in_bat_obs, out.Common.stop)


class ObserveSocketOnBattery(State, BaseState):

    def __init__(self, config: dict[str, Any], action_srv: SimpleActionServer):
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_bat_pre_connect],
                       output_keys=['xyz_xyzw_base2socket'])
        BaseState.__init__(self, config, action_srv)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Observe socket on battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.DisconnectFromCar.plug_in_bat_pre_connect, out.Common.stop)


class InsertPlugToBattery(State, BaseState):

    def __init__(self, config: dict[str, Any], action_srv: SimpleActionServer):
        State.__init__(self, 
                       outcomes=[out.Common.stop, out.DisconnectFromCar.plug_in_bat_connect],
                       input_keys=['xyz_xyzw_base2socket'])
        BaseState.__init__(self, config, action_srv)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Insert plug to battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.DisconnectFromCar.plug_in_bat_connect, out.Common.stop)


class ReleasePlugOnBattery(State, BaseState):

    def __init__(self, config: dict[str, Any], action_srv: SimpleActionServer):
        State.__init__(self, outcomes=[out.Common.stop, out.DisconnectFromCar.arm_in_bat_post_connect])
        BaseState.__init__(self, config, action_srv)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Release plug on battery')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.DisconnectFromCar.arm_in_bat_post_connect, out.Common.stop)


class MoveArmToDrivePos(State, BaseState):

    def __init__(self, config: dict[str, Any], action_srv: SimpleActionServer):
        State.__init__(self, outcomes=[out.Common.stop, out.DisconnectFromCar.arm_in_driving_pose])
        BaseState.__init__(self, config, action_srv)

    def execute(self, ud: Any) -> str:
        rospy.loginfo('Move arm to drive pos')

        # --- Add your magic here --- #
        time.sleep(_time_out)
        # --------------------------- #
        return self._user_request(out.DisconnectFromCar.arm_in_driving_pose, out.Common.stop)
