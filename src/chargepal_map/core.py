from __future__ import annotations

# global
import abc
from pathlib import Path
from ur_pilot import Pilot
from smach import StateMachine

import chargepal_map.processes.outcomes as out
import chargepal_map.processes.connect_to_car as ctc
from chargepal_map.processes.utils import state_name


class Process(metaclass=abc.ABCMeta):

    def __init__(self, cfg_dir: Path, pilot: Pilot):
        self.cfg_dir = cfg_dir
        self.pilot = pilot

    @abc.abstractmethod
    def get_process(self) -> StateMachine:
        raise NotImplementedError("Must be implemented in child class")


class ConnectToCarProcess(Process):

    def __init__(self, cfg_dir: Path, pilot: Pilot) -> None:
        super().__init__(cfg_dir, pilot)
        
        self.process = StateMachine(outcomes=[out.ConnectToCar.arm_in_driving_pose])
        # Open smash container to add states and transitions
        with self.ctc_process:
            StateMachine.add(
                label=state_name(ctc.MoveArmToBattery), 
                state=ctc.MoveArmToBattery(self._ur_pilot), 
                transitions={out.ConnectToCar.arm_in_bat_obs:state_name(ctc.ObservePlugOnBattery)}
                )
            StateMachine.add(
                label=state_name(ctc.ObservePlugOnBattery), 
                state=ctc.ObservePlugOnBattery(self._ur_pilot), 
                transitions={out.ConnectToCar.arm_in_bat_pre_connect:state_name(ctc.GraspPlugOnBattery)}
                )
            StateMachine.add(
                label=state_name(ctc.GraspPlugOnBattery), 
                state=ctc.GraspPlugOnBattery(self._ur_pilot), 
                transitions={out.ConnectToCar.plug_in_bat_connect:state_name(ctc.RemovePlugFromBattery)}
                )
            StateMachine.add(
                label=state_name(ctc.RemovePlugFromBattery), 
                state=ctc.RemovePlugFromBattery(self._ur_pilot), 
                transitions={out.ConnectToCar.plug_in_bat_post_connect:state_name(ctc.MovePlugToCar)}
                )
            StateMachine.add(
                label=state_name(ctc.MovePlugToCar), 
                state=ctc.MovePlugToCar(self._ur_pilot), 
                transitions={out.ConnectToCar.plug_in_car_obs:state_name(ctc.ObserveSocketOnCar)}
                )
            StateMachine.add(
                label=state_name(ctc.ObserveSocketOnCar), 
                state=ctc.ObserveSocketOnCar(self._ur_pilot), 
                transitions={out.ConnectToCar.plug_in_car_pre_connect:state_name(ctc.InsertPlugToCar)}
                )
            StateMachine.add(
                label=state_name(ctc.InsertPlugToCar), 
                state=ctc.InsertPlugToCar(self._ur_pilot), 
                transitions={out.ConnectToCar.plug_in_car_connect:state_name(ctc.ReleasePlugOnCar)}
                )
            StateMachine.add(
                label=state_name(ctc.ReleasePlugOnCar), 
                state=ctc.ReleasePlugOnCar(self._ur_pilot), 
                transitions={out.ConnectToCar.arm_in_car_post_connect:state_name(ctc.MoveArmToDrivePos)}
                )
            StateMachine.add(
                label=state_name(ctc.MoveArmToDrivePos), 
                state=ctc.MoveArmToDrivePos(self._ur_pilot), 
                # No transition. Stop when reaching this state
                )

    def get_process(self) -> StateMachine:
        return self.process



class ProcessFactory:

    _process_builder = {
        'connect_to_car': ConnectToCarProcess, 
    }

    @staticmethod
    def create(proc_name: str, cfg_dir: Path, ur_pilot: Pilot) -> StateMachine:
        builder = ProcessFactory._process_builder[proc_name](cfg_dir=cfg_dir, pilot=ur_pilot)
        return builder.get_process()

