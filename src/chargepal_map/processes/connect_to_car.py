from __future__ import annotations

# global
from smach import State
from ur_pilot import Pilot

# local
import chargepal_map.processes.outcomes as out


class MoveArmToBattery(State):

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_bat_obs])
        self._pilot = pilot

    def execute(self, ud):
        self._pilot.move_to_joint_pos([0, 0, 0, 0, 0, 0])
        return out.ConnectToCar.arm_in_bat_obs


class ObservePlugOnBattery(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_bat_pre_connect])
        self._pilot = pilot

    def execute(self, ud):
        self._pilot.move_to_joint_pos([0, 0, 0, 0, 0, 0])
        return out.ConnectToCar.arm_in_bat_pre_connect


class GraspPlugOnBattery(State):

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_bat_connect])
        self._pilot = pilot

    def execute(self, ud):
        self._pilot.move_to_joint_pos([0, 0, 0, 0, 0, 0])
        return out.ConnectToCar.plug_in_bat_connect
    

class RemovePlugFromBattery(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_bat_post_connect])
        self._pilot = pilot

    def execute(self, ud):
        self._pilot.move_to_joint_pos([0, 0, 0, 0, 0, 0])
        return out.ConnectToCar.plug_in_bat_post_connect


class MovePlugToCar(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_obs])
        self._pilot = pilot

    def execute(self, ud):
        self._pilot.move_to_joint_pos([0, 0, 0, 0, 0, 0])
        return out.ConnectToCar.plug_in_car_obs


class ObserveSocketOnCar(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_pre_connect])
        self._pilot = pilot

    def execute(self, ud):
        self._pilot.move_to_joint_pos([0, 0, 0, 0, 0, 0])
        return out.ConnectToCar.plug_in_car_pre_connect


class InsertPlugToCar(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_connect])
        self._pilot = pilot

    def execute(self, ud):
        self._pilot.move_to_joint_pos([0, 0, 0, 0, 0, 0])
        return out.ConnectToCar.plug_in_car_connect


class ReleasePlugOnCar(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_car_post_connect])
        self._pilot = pilot

    def execute(self, ud):
        self._pilot.move_to_joint_pos([0, 0, 0, 0, 0, 0])
        return out.ConnectToCar.arm_in_car_post_connect


class MoveArmToDrivePos(State):

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_driving_pose])
        self._pilot = pilot

    def execute(self, ud):
        self._pilot.move_to_joint_pos([0, 0, 0, 0, 0, 0])
        return out.ConnectToCar.arm_in_driving_pose
