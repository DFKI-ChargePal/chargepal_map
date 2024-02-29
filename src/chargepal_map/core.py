from __future__ import annotations

# global
from pathlib import Path

# local
from chargepal_map.state_machine.processes.process import ProcessABC
from chargepal_map.state_machine.processes.connect_to_car import ConnectToCarTwist, ConnectToCarTwistEasy
from chargepal_map.state_machine.processes.disconnect_from_car import DisconnectFromCarTwist, DisconnectFromCarTwistEasy

# typing
from typing import Type
from ur_pilot import Pilot


class ProcessFactory:

    def __init__(self) -> None:
        self._selection: dict[str, Type[ProcessABC]] = {}

    def register(self, name: str, process: Type[ProcessABC]) -> None:
        self._selection[name] = process

    def create(self, name: str, cfg_fp: Path, pilot: Pilot, detector_dir: Path) -> ProcessABC:
        proc_type = self._selection.get(name)
        if proc_type is None:
            raise KeyError(f"Unknown or unregistered process '{name}'. " 
                           f"Available processes are: {list(self._selection.keys())}")
        builder = proc_type(name=name, cfg_fp=cfg_fp, dtt_dir=detector_dir)
        builder.set_up(pilot)
        return builder


manipulation_action_processor = ProcessFactory()

manipulation_action_processor.register('connect_to_car_twist', ConnectToCarTwist)
# manipulation_action_processor.register('connect_to_car_electric', ConnectToCarElectric)
manipulation_action_processor.register('connect_to_car_twist_easy', ConnectToCarTwistEasy)

manipulation_action_processor.register('disconnect_from_car_twist', DisconnectFromCarTwist)
# manipulation_action_processor.register('disconnect_from_car_electric', DisconnectFromCarElectric)
manipulation_action_processor.register('disconnect_from_car_twist_easy', DisconnectFromCarTwistEasy)
