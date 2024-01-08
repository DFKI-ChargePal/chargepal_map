from __future__ import annotations

# global
import abc
import yaml
import rospy
from pathlib import Path
from smach import StateMachine

# local
import chargepal_map.state_machine.outcomes as out
import chargepal_map.state_machine.connect_to_car as ctc
import chargepal_map.state_machine.disconnect_from_car as dfc
from chargepal_map.state_machine.utils import (
    state_name, 
    silent_smach
)

# typing 
from typing import Any, Type


class StateMachineBuilder(metaclass=abc.ABCMeta):

    def __init__(self, cfg_fp: Path):
        silent_smach()
        if not cfg_fp.exists():
            raise FileNotFoundError(f"Can't find configuration file under: {cfg_fp}")
        # load configuration file
        with cfg_fp.open('r') as file_stream:
            try:
                self.config: dict[str, dict[str, Any]] = yaml.safe_load(file_stream)
            except Exception as e:
                raise RuntimeError(f"Error while reading {cfg_fp.name} configuration. {e}")
        if self.config is None:
            rospy.logwarn(f"Empty configuration file: {cfg_fp.name}")
            self.config = {}

    @abc.abstractmethod
    def set_up(self) -> StateMachine:
        raise NotImplementedError("Must be implemented in child class")


class ConnectToCar(StateMachineBuilder):

    def __init__(self, cfg_fp: Path) -> None:
        super().__init__(cfg_fp)
        self.state_machine = StateMachine(outcomes=[out.ConnectToCar.arm_in_driving_pose])

    def set_up(self) -> StateMachine:
        # Open smash container to add states and transitions
        with self.state_machine:
            StateMachine.add(
                label=state_name(ctc.MoveArmToBattery),
                state=ctc.MoveArmToBattery(self.config),
                transitions={out.ConnectToCar.arm_in_bat_obs: state_name(ctc.ObservePlugOnBattery)}
            )
            StateMachine.add(
                label=state_name(ctc.ObservePlugOnBattery),
                state=ctc.ObservePlugOnBattery(self.config),
                transitions={out.ConnectToCar.arm_in_bat_pre_connect: state_name(ctc.GraspPlugOnBattery)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc.GraspPlugOnBattery),
                state=ctc.GraspPlugOnBattery(self.config),
                transitions={out.ConnectToCar.plug_in_bat_connect: state_name(ctc.RemovePlugFromBattery)}
            )
            StateMachine.add(
                label=state_name(ctc.RemovePlugFromBattery),
                state=ctc.RemovePlugFromBattery(self.config),
                transitions={out.ConnectToCar.plug_in_bat_post_connect: state_name(ctc.MovePlugToCar)}
            )
            StateMachine.add(
                label=state_name(ctc.MovePlugToCar),
                state=ctc.MovePlugToCar(self.config),
                transitions={out.ConnectToCar.plug_in_car_obs: state_name(ctc.ObserveSocketOnCar)}
            )
            StateMachine.add(
                label=state_name(ctc.ObserveSocketOnCar),
                state=ctc.ObserveSocketOnCar(self.config),
                transitions={out.ConnectToCar.plug_in_car_pre_connect: state_name(ctc.InsertPlugToCar)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc.InsertPlugToCar),
                state=ctc.InsertPlugToCar(self.config),
                transitions={out.ConnectToCar.plug_in_car_connect: state_name(ctc.ReleasePlugOnCar)}
            )
            StateMachine.add(
                label=state_name(ctc.ReleasePlugOnCar),
                state=ctc.ReleasePlugOnCar(self.config),
                transitions={out.ConnectToCar.arm_in_car_post_connect: state_name(ctc.MoveArmToDrivePos)}
            )
            StateMachine.add(
                label=state_name(ctc.MoveArmToDrivePos),
                state=ctc.MoveArmToDrivePos(self.config),
                # No transition. Stop when reaching this state
            )
        return self.state_machine


class DisconnectFromCar(StateMachineBuilder):

    def __init__(self, cfg_fp: Path) -> None:
        super().__init__(cfg_fp)
        self.state_machine = StateMachine(outcomes=[out.DisconnectFromCar.arm_in_driving_pose])

    def set_up(self) -> StateMachine:
        # Open smash container to add states and transitions
        with self.state_machine:
            StateMachine.add(
                label=state_name(dfc.MoveArmToCar),
                state=dfc.MoveArmToCar(self.config),
                transitions={out.DisconnectFromCar.arm_in_car_obs: state_name(dfc.ObservePlugOnCar)}
            )
            StateMachine.add(
                label=state_name(dfc.ObservePlugOnCar),
                state=dfc.ObservePlugOnCar(self.config),
                transitions={out.DisconnectFromCar.arm_in_car_pre_connect: state_name(dfc.GraspPlugOnCar)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc.GraspPlugOnCar),
                state=dfc.GraspPlugOnCar(self.config),
                transitions={out.DisconnectFromCar.plug_in_car_connect: state_name(dfc.RemovePlugFromCar)}
            )
            StateMachine.add(
                label=state_name(dfc.RemovePlugFromCar),
                state=dfc.RemovePlugFromCar(self.config),
                transitions={out.DisconnectFromCar.plug_in_car_post_connect: state_name(dfc.MovePlugToBattery)}
            )
            StateMachine.add(
                label=state_name(dfc.MovePlugToBattery),
                state=dfc.MovePlugToBattery(self.config),
                transitions={out.DisconnectFromCar.plug_in_bat_obs: state_name(dfc.ObserveSocketOnBattery)}
            )
            StateMachine.add(
                label=state_name(dfc.ObserveSocketOnBattery),
                state=dfc.ObserveSocketOnBattery(self.config),
                transitions={out.DisconnectFromCar.plug_in_bat_pre_connect: state_name(dfc.InsertPlugToBattery)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc.InsertPlugToBattery),
                state=dfc.InsertPlugToBattery(self.config),
                transitions={out.DisconnectFromCar.plug_in_bat_connect: state_name(dfc.ReleasePlugOnBattery)}
            )
            StateMachine.add(
                label=state_name(dfc.ReleasePlugOnBattery),
                state=dfc.ReleasePlugOnBattery(self.config),
                transitions={out.DisconnectFromCar.arm_in_bat_post_connect: state_name(dfc.MoveArmToDrivePos)}
            )
            StateMachine.add(
                label=state_name(dfc.MoveArmToDrivePos),
                state=dfc.MoveArmToDrivePos(self.config),
                # No transition. Stop when reaching this state
            )
        return self.state_machine


class ProcessFactory:

    def __init__(self) -> None:
        self._selection: dict[str, Type[StateMachineBuilder]] = {}

    def register_process(self, name: str, process: Type[StateMachineBuilder]) -> None:
        self._selection[name] = process

    def create(self, name: str, cfg_dir: Path) -> StateMachine:
        cfg_fp = cfg_dir.joinpath(name + '.yaml')
        builder = self._selection[name](cfg_fp=cfg_fp)
        return builder.set_up()


manipulation_action_processor = ProcessFactory()
manipulation_action_processor.register_process('connect_to_car', ConnectToCar)
manipulation_action_processor.register_process('disconnect_from_car', DisconnectFromCar)
