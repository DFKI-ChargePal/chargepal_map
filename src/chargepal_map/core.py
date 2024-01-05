from __future__ import annotations

# global
import abc
from pathlib import Path
from smach import StateMachine

import chargepal_map.state_machine.outcomes as out
import chargepal_map.state_machine.connect_to_car as ctc
import chargepal_map.state_machine.disconnect_from_car as dfc
from chargepal_map.state_machine.utils import state_name, silent_smach


class StateMachineBuilder(metaclass=abc.ABCMeta):

    def __init__(self, cfg_dir: Path):
        silent_smach()
        self.cfg_dir = cfg_dir

    @abc.abstractmethod
    def set_up(self) -> StateMachine:
        raise NotImplementedError("Must be implemented in child class")


class ConnectToCar(StateMachineBuilder):

    def __init__(self, cfg_dir: Path) -> None:
        super().__init__(cfg_dir)
        self.state_machine = StateMachine(outcomes=[out.ConnectToCar.arm_in_driving_pose])

    def set_up(self) -> StateMachine:
        # Open smash container to add states and transitions
        with self.state_machine:
            StateMachine.add(
                label=state_name(ctc.MoveArmToBattery),
                state=ctc.MoveArmToBattery(self.cfg_dir),
                transitions={out.ConnectToCar.arm_in_bat_obs: state_name(ctc.ObservePlugOnBattery)}
            )
            StateMachine.add(
                label=state_name(ctc.ObservePlugOnBattery),
                state=ctc.ObservePlugOnBattery(self.cfg_dir),
                transitions={out.ConnectToCar.arm_in_bat_pre_connect: state_name(ctc.GraspPlugOnBattery)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc.GraspPlugOnBattery),
                state=ctc.GraspPlugOnBattery(self.cfg_dir),
                transitions={out.ConnectToCar.plug_in_bat_connect: state_name(ctc.RemovePlugFromBattery)}
            )
            StateMachine.add(
                label=state_name(ctc.RemovePlugFromBattery),
                state=ctc.RemovePlugFromBattery(self.cfg_dir),
                transitions={out.ConnectToCar.plug_in_bat_post_connect: state_name(ctc.MovePlugToCar)}
            )
            StateMachine.add(
                label=state_name(ctc.MovePlugToCar),
                state=ctc.MovePlugToCar(self.cfg_dir),
                transitions={out.ConnectToCar.plug_in_car_obs: state_name(ctc.ObserveSocketOnCar)}
            )
            StateMachine.add(
                label=state_name(ctc.ObserveSocketOnCar),
                state=ctc.ObserveSocketOnCar(self.cfg_dir),
                transitions={out.ConnectToCar.plug_in_car_pre_connect: state_name(ctc.InsertPlugToCar)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc.InsertPlugToCar),
                state=ctc.InsertPlugToCar(self.cfg_dir),
                transitions={out.ConnectToCar.plug_in_car_connect: state_name(ctc.ReleasePlugOnCar)}
            )
            StateMachine.add(
                label=state_name(ctc.ReleasePlugOnCar),
                state=ctc.ReleasePlugOnCar(self.cfg_dir),
                transitions={out.ConnectToCar.arm_in_car_post_connect: state_name(ctc.MoveArmToDrivePos)}
            )
            StateMachine.add(
                label=state_name(ctc.MoveArmToDrivePos),
                state=ctc.MoveArmToDrivePos(self.cfg_dir),
                # No transition. Stop when reaching this state
            )
        return self.state_machine


class DisconnectFromCar(StateMachineBuilder):

    def __init__(self, cfg_dir: Path) -> None:
        super().__init__(cfg_dir)
        self.state_machine = StateMachine(outcomes=[out.DisconnectFromCar.arm_in_driving_pose])

    def set_up(self) -> StateMachine:
        # Open smash container to add states and transitions
        with self.state_machine:
            StateMachine.add(
                label=state_name(dfc.MoveArmToCar),
                state=dfc.MoveArmToCar(self.cfg_dir),
                transitions={out.DisconnectFromCar.arm_in_car_obs: state_name(dfc.ObservePlugOnCar)}
            )
            StateMachine.add(
                label=state_name(dfc.ObservePlugOnCar),
                state=dfc.ObservePlugOnCar(self.cfg_dir),
                transitions={out.DisconnectFromCar.arm_in_car_pre_connect: state_name(dfc.GraspPlugOnCar)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc.GraspPlugOnCar),
                state=dfc.GraspPlugOnCar(self.cfg_dir),
                transitions={out.DisconnectFromCar.plug_in_car_connect: state_name(dfc.RemovePlugFromCar)}
            )
            StateMachine.add(
                label=state_name(dfc.RemovePlugFromCar),
                state=dfc.RemovePlugFromCar(self.cfg_dir),
                transitions={out.DisconnectFromCar.plug_in_car_post_connect: state_name(dfc.MovePlugToBattery)}
            )
            StateMachine.add(
                label=state_name(dfc.MovePlugToBattery),
                state=dfc.MovePlugToBattery(self.cfg_dir),
                transitions={out.DisconnectFromCar.plug_in_bat_obs: state_name(dfc.ObserveSocketOnBattery)}
            )
            StateMachine.add(
                label=state_name(dfc.ObserveSocketOnBattery),
                state=dfc.ObserveSocketOnBattery(self.cfg_dir),
                transitions={out.DisconnectFromCar.plug_in_bat_pre_connect: state_name(dfc.InsertPlugToBattery)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc.InsertPlugToBattery),
                state=dfc.InsertPlugToBattery(self.cfg_dir),
                transitions={out.DisconnectFromCar.plug_in_bat_connect: state_name(dfc.ReleasePlugOnBattery)}
            )
            StateMachine.add(
                label=state_name(dfc.ReleasePlugOnBattery),
                state=dfc.ReleasePlugOnBattery(self.cfg_dir),
                transitions={out.DisconnectFromCar.arm_in_bat_post_connect: state_name(dfc.MoveArmToDrivePos)}
            )
            StateMachine.add(
                label=state_name(dfc.MoveArmToDrivePos),
                state=dfc.MoveArmToDrivePos(self.cfg_dir),
                # No transition. Stop when reaching this state
            )
        return self.state_machine


class ProcessFactory:

    def __init__(self) -> None:
        self._selection = {
            'connect_to_car': ConnectToCar,
            'disconnect_from_car': DisconnectFromCar,
            }

    def create(self, name: str, cfg_dir: Path) -> StateMachine:
        builder = self._selection[name](cfg_dir=cfg_dir)
        return builder.set_up()


manipulation_action_processor = ProcessFactory()
