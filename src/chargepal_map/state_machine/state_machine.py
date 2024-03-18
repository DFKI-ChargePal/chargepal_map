from __future__ import annotations

# libs
import ur_pilot
import yaml
import rospy
from pathlib import Path
from smach import StateMachine

import chargepal_map.state_machine.states as s
from chargepal_map.state_machine.outcomes import out
from chargepal_map.state_machine.utils import silent_smach, state_name

# typing
from typing import Any
from ur_pilot import Pilot



class ManipulationStateMachine:

    def __init__(self, cfg_fp: Path, dtt_dir: Path):
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
        self.state_machine = StateMachine(outcomes=[out.stop, out.completed], input_keys=['job'])

    def build(self, pilot: Pilot) -> None:
        with self.state_machine:
            StateMachine.add(
                label=state_name(s.Start),
                state=s.Start(self.config, pilot),
                transitions={
                    out.arm_in_wrong_ws: state_name(s.FlipArm),
                    out.arm_ready_do:    state_name(s.MoveToPlugPreObs),
                    out.arm_ready_no:    state_name(s.MoveToPlugPreAttached),
                    out.stop:            state_name(s.Stop)
                },
                remapping={'job': 'job'}
            )
            StateMachine.add(
                label=state_name(s.FlipArm),
                state=s.FlipArm(self.config, pilot),
                transitions={
                    out.arm_ready_do: state_name(s.MoveToPlugPreObs),
                    out.arm_ready_no: state_name(s.MoveToPlugPreAttached),
                    out.stop:         state_name(s.Stop)
                },
                remapping={'job': 'job'}
            )
            StateMachine.add(
                label=state_name(s.MoveToPlugPreObs),
                state=s.MoveToPlugPreObs(self.config, pilot),
                transitions={
                    out.plug_pre_obs: state_name(s.ObservePlug),
                    out.stop:         state_name(s.Stop)
                },
                remapping={'job': 'job'}
            )
            StateMachine.add(
                label=state_name(s.MoveToPlugPreAttached),
                state=s.MoveToPlugPreAttached(self.config, pilot),
                transitions={
                    out.plug_pre_attached: state_name(s.AttachPlug),
                    out.stop:              state_name(s.Stop)
                },
                remapping={'job': 'job'}
            )
            StateMachine.add(
                label=state_name(s.AttachPlug),
                state=s.AttachPlug(self.config, pilot),
                transitions={
                    out.plug_attached: state_name(s.RemovePlug),
                    out.stop:          state_name(s.Stop)
                },
                remapping={'job': 'job'}
            )
            StateMachine.add(
                label=state_name(s.RemovePlug),
                state=s.RemovePlug(self.config, pilot),
                transitions={
                    out.plug_removed_do: state_name(s.MoveToSocketPreObs),
                    out.plug_removed_no: state_name(s.MoveToPlugPreConnected),
                    out.stop:            state_name(s.Stop)
                },
                remapping={'job': 'job'}
            )
            StateMachine.add(
                label=state_name(s.MoveToSocketPreObs),
                state=s.MoveToSocketPreObs(self.config, pilot),
                transitions={
                    out.socket_pre_obs: state_name(s.ObserveSocket),
                    out.stop:           state_name(s.Stop)
                },
                remapping={'job': 'job'}
            )
            StateMachine.add(
                label=state_name(s.ObserveSocket),
                state=s.ObserveSocket(self.config, pilot),
                transitions={
                    out.socket_obs: state_name(s.MoveToPlugPreConnected),
                    out.stop:       state_name(s.Stop)
                },
                remapping={'job': 'job'}
            )
            StateMachine.add(
                label=state_name(s.MoveToPlugPreConnected),
                state=s.MoveToPlugPreConnected(self.config, pilot),
                transitions={
                    out.plug_pre_connected: state_name(s.InsertPlug),
                    out.stop:               state_name(s.Stop)
                },
                remapping={'job': 'job'}
            )
            StateMachine.add(
                label=state_name(s.InsertPlug),
                state=s.InsertPlug(self.config, pilot),
                transitions={
                    out.plug_connected: state_name(s.ReleasePlug),
                    out.stop:           state_name(s.Stop)
                },
                remapping={'job': 'job'}
            )
            StateMachine.add(
                label=state_name(s.ReleasePlug),
                state=s.ReleasePlug(self.config, pilot),
                transitions={
                    out.plug_released: state_name(s.MoveToWs),
                    out.stop:          state_name(s.Stop)
                },
                remapping={'job': 'job'}
            )
            StateMachine.add(
                label=state_name(s.MoveToWs),
                state=s.MoveToWs(self.config, pilot),
                transitions={
                    out.completed: out.completed
                },
            )
            StateMachine.add(
                label=state_name(s.Stop),
                state=s.Stop(self.config, pilot),
                transitions={out.stop: out.stop}
            )
