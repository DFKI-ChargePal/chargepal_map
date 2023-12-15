from __future__ import annotations

# global
import time
from smach import State
from ur_pilot import Pilot

# local
import chargepal_map.processes.outcomes as out

_time_out = 2.0


class MoveArmToBattery(State):

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_bat_obs])
        self._pilot = pilot

    def execute(self, ud):
        with self._pilot.position_control():    
            self._pilot.move_to_joint_pos([3.773, -1.159, 1.914, -0.263, 2.073, -1.267])
        time.sleep(_time_out)
        return out.ConnectToCar.arm_in_bat_obs


class ObservePlugOnBattery(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_bat_pre_connect])
        self._pilot = pilot

    def execute(self, ud):
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.670, -0.987, 1.704, -0.154, 1.980, -1.267])
        time.sleep(_time_out)
        return out.ConnectToCar.arm_in_bat_pre_connect


class GraspPlugOnBattery(State):

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_bat_connect])
        self._pilot = pilot

    def execute(self, ud):
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.634, -0.917, 1.634, -0.152, 1.977, -1.267])
        time.sleep(_time_out)
        return out.ConnectToCar.plug_in_bat_connect
    

class RemovePlugFromBattery(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_bat_post_connect])
        self._pilot = pilot

    def execute(self, ud):
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.663, -1.045, 1.736, -0.127, 1.978, -1.267])
        time.sleep(_time_out)
        return out.ConnectToCar.plug_in_bat_post_connect


class MovePlugToCar(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_obs])
        self._pilot = pilot

    def execute(self, ud):
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.476, -1.500, 1.756, 0.050, 1.887, -1.267])
        time.sleep(_time_out)
        return out.ConnectToCar.plug_in_car_obs


class ObserveSocketOnCar(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_pre_connect])
        self._pilot = pilot

    def execute(self, ud):
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.346, -1.467, 1.835, -0.363, 1.766, -1.578])
        time.sleep(_time_out)
        return out.ConnectToCar.plug_in_car_pre_connect


class InsertPlugToCar(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.plug_in_car_connect])
        self._pilot = pilot

    def execute(self, ud):
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.337, -1.412, 1.761, -0.362, 1.766, -1.578])
        time.sleep(_time_out)
        return out.ConnectToCar.plug_in_car_connect


class ReleasePlugOnCar(State):
    
    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_car_post_connect])
        self._pilot = pilot

    def execute(self, ud):
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.334, -1.451, 1.859, -0.422, 1.770, -1.578])
        time.sleep(_time_out)
        return out.ConnectToCar.arm_in_car_post_connect


class MoveArmToDrivePos(State):

    def __init__(self, pilot: Pilot):
        super().__init__(outcomes=[out.ConnectToCar.arm_in_driving_pose])
        self._pilot = pilot

    def execute(self, ud):
        with self._pilot.position_control():
            self._pilot.move_to_joint_pos([3.431, -1.371, 2.129, -0.743, 1.869, -1.562])
        time.sleep(_time_out)
        return out.ConnectToCar.arm_in_driving_pose
