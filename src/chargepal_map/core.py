from __future__ import annotations

# global
import abc
from pathlib import Path
from ur_pilot import Pilot
from smach import StateMachine

import chargepal_map.processes.outcomes as out
import chargepal_map.processes.connect_to_car as ctc
import chargepal_map.processes.disconnect_from_car as dfc
from chargepal_map.processes.utils import state_name


class Process(metaclass=abc.ABCMeta):

    def __init__(self, cfg_dir: Path, pilot: Pilot):
        self.cfg_dir = cfg_dir
        self._ur_pilot = pilot

    @abc.abstractmethod
    def _get_process(self) -> StateMachine:
        raise NotImplementedError("Must be implemented in child class")


class ConnectToCarProcess(Process):

    def __init__(self, cfg_dir: Path, pilot: Pilot) -> None:
        super().__init__(cfg_dir, pilot)
        
        self.process = StateMachine(outcomes=[out.ConnectToCar.arm_in_driving_pose])
        # Open smash container to add states and transitions
        with self.process:
            StateMachine.add(
                label=state_name(ctc.MoveArmToBattery), 
                state=ctc.MoveArmToBattery(self._ur_pilot), 
                transitions={out.ConnectToCar.arm_in_bat_obs:state_name(ctc.ObservePlugOnBattery)}
                )
            StateMachine.add(
                label=state_name(ctc.ObservePlugOnBattery), 
                state=ctc.ObservePlugOnBattery(self._ur_pilot), 
                transitions={out.ConnectToCar.arm_in_bat_pre_connect:state_name(ctc.GraspPlugOnBattery)},
                remapping={'xyz_xyzw_base2socket':'xyz_xyzw_base2socket'}
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
                transitions={out.ConnectToCar.plug_in_car_pre_connect:state_name(ctc.InsertPlugToCar)},
                remapping={'xyz_xyzw_base2socket':'xyz_xyzw_base2socket'}
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

    def _get_process(self) -> StateMachine:
        return self.process


class DisconnectFromCarProcess(Process):

    def __init__(self, cfg_dir: Path, pilot: Pilot) -> None:
        super().__init__(cfg_dir, pilot)
        
        self.process = StateMachine(outcomes=[out.DisconnectFromCar.arm_in_driving_pose])
        # Open smash container to add states and transitions
        with self.process:
            StateMachine.add(
                label=state_name(dfc.MoveArmToCar), 
                state=dfc.MoveArmToCar(self._ur_pilot), 
                transitions={out.DisconnectFromCar.arm_in_car_obs:state_name(dfc.ObservePlugOnCar)}
                )
            StateMachine.add(
                label=state_name(dfc.ObservePlugOnCar), 
                state=dfc.ObservePlugOnCar(self._ur_pilot), 
                transitions={out.DisconnectFromCar.arm_in_car_pre_connect:state_name(dfc.GraspPlugOnCar)},
                remapping={'xyz_xyzw_base2socket':'xyz_xyzw_base2socket'}
                )
            StateMachine.add(
                label=state_name(dfc.GraspPlugOnCar), 
                state=dfc.GraspPlugOnCar(self._ur_pilot),
                transitions={out.DisconnectFromCar.plug_in_car_connect:state_name(dfc.RemovePlugFromCar)}
                )
            StateMachine.add(
                label=state_name(dfc.RemovePlugFromCar), 
                state=dfc.RemovePlugFromCar(self._ur_pilot), 
                transitions={out.DisconnectFromCar.plug_in_car_post_connect:state_name(dfc.MovePlugToBattery)}
                )
            StateMachine.add(
                label=state_name(dfc.MovePlugToBattery), 
                state=dfc.MovePlugToBattery(self._ur_pilot), 
                transitions={out.DisconnectFromCar.plug_in_bat_obs:state_name(dfc.ObserveSocketOnBattery)}
                )
            StateMachine.add(
                label=state_name(dfc.ObserveSocketOnBattery), 
                state=dfc.ObserveSocketOnBattery(self._ur_pilot), 
                transitions={out.DisconnectFromCar.plug_in_bat_pre_connect:state_name(dfc.InsertPlugToBattery)},
                remapping={'xyz_xyzw_base2socket':'xyz_xyzw_base2socket'}
                )
            StateMachine.add(
                label=state_name(dfc.InsertPlugToBattery), 
                state=dfc.InsertPlugToBattery(self._ur_pilot), 
                transitions={out.DisconnectFromCar.plug_in_bat_connect:state_name(dfc.ReleasePlugOnBattery)}
                )
            StateMachine.add(
                label=state_name(dfc.ReleasePlugOnBattery), 
                state=dfc.ReleasePlugOnBattery(self._ur_pilot), 
                transitions={out.DisconnectFromCar.arm_in_bat_post_connect:state_name(dfc.MoveArmToDrivePos)}
                )
            StateMachine.add(
                label=state_name(dfc.MoveArmToDrivePos), 
                state=dfc.MoveArmToDrivePos(self._ur_pilot), 
                # No transition. Stop when reaching this state
                )

    def _get_process(self) -> StateMachine:
        return self.process


class ProcessFactory:

    def __init__(self) -> None:
        self._process_builder = {
            'connect_to_car': ConnectToCarProcess,
            'disconnect_from_car': DisconnectFromCarProcess,
            }

    def get_process(self, proc_name: str, cfg_dir: Path, ur_pilot: Pilot) -> StateMachine:
        builder = self._process_builder[proc_name](cfg_dir=cfg_dir, pilot=ur_pilot)
        return builder._get_process()


factory = ProcessFactory()
