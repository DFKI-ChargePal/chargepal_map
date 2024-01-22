from __future__ import annotations

# global
import abc
import yaml
import rospy
from pathlib import Path

# local
from chargepal_map.state_machine.utils import silent_smach

# typing
from typing import Any
from ur_pilot import Pilot



class ProcessABC(metaclass=abc.ABCMeta):

    def __init__(self, name: str, cfg_fp: Path, dtt_dir: Path):
        """ Base class of a manipulation process

        Args:
            name:    Name of the Process
            cfg_fp:  File path to the process configuration file
            dtt_dir: Directory path to the vision detector configurations

        Raises:
            FileNotFoundError: _description_
            RuntimeError: _description_
        """
        silent_smach()
        self.name = name
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
        self.config.setdefault('common', {})
        if not dtt_dir.exists():
            raise NotADirectoryError(f"Can't find detector directory under: {dtt_dir}")
        self.config['common']['detector_dir'] = dtt_dir

    @abc.abstractmethod
    def action_callback(self, goal: Any) -> None:
        raise NotImplementedError("Must be implemented in child class")

    @abc.abstractmethod
    def wait_for_usr_feedback(self) -> None:
        raise NotImplementedError("Must be implemented in child class")

    @abc.abstractmethod
    def set_up(self, pilot: Pilot) -> None:
        raise NotImplementedError("Must be implemented in child class")
