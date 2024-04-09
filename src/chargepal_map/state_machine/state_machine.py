from __future__ import annotations

# libs
import copy
from pathlib import Path
from smach import StateMachine

import chargepal_map.state_machine.states as s
from chargepal_map.state_machine.outcomes import out
from chargepal_map.state_machine.step_by_user import StepByUserServer
from chargepal_map.state_machine.utils import silent_smach, state_name

# typing
from typing import Any
from ur_pilot import Pilot


class ManipulationStateMachine:

    def __init__(self, config_dir: Path, base_cfg: dict[str, Any]):
        """ Base class of a manipulation process

        Args:
            config_dir: Path pointing to the configuration folder
            base_cfg:   Top-level configuration dictionary

        """
        silent_smach()
        # Get available configuration files
        self.config = copy.deepcopy(base_cfg['state_machine'])
        self.config.setdefault('step_by_user', False)
        # Match detector file names with configuration file paths
        self.config['detector']['files'] = {
            cfg_fp.stem: cfg_fp
            for cfg_fp in config_dir.joinpath(self.config['detector']['folder_name']).glob('*.yaml')
            }
        # Match state file names with configuration file paths
        self.config['states']['files'] = {
            cfg_fp.stem: cfg_fp 
            for cfg_fp in config_dir.joinpath(self.config['states']['folder_name']).glob('*.yaml')
            }
        self.usr_srv = StepByUserServer()
        self.state_machine = StateMachine(outcomes=[out.stop, out.completed], input_keys=['job'])

    def build(self, pilot: Pilot) -> None:
        print(f"Hello state machine")
        with self.state_machine:
            print(f"Add state Start")
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
            print(f"Add state Flip arm")
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
                label=state_name(s.ObservePlug),
                state=s.ObservePlug(self.config, pilot),
                transitions={
                    out.plug_obs: state_name(s.MoveToPlugPreAttached),
                    out.stop:     state_name(s.Stop)
                },
                remapping={
                    'job': 'job',
                    'T_base2socket': 'T_base2socket',
                }
            )
            StateMachine.add(
                label=state_name(s.MoveToPlugPreAttached),
                state=s.MoveToPlugPreAttached(self.config, pilot),
                transitions={
                    out.plug_pre_attached: state_name(s.AttachPlug),
                    out.stop:              state_name(s.Stop)
                },
                remapping={
                    'job': 'job',
                    'T_base2socket': 'T_base2socket',
                }
            )
            StateMachine.add(
                label=state_name(s.AttachPlug),
                state=s.AttachPlug(self.config, pilot),
                transitions={
                    out.plug_attached: state_name(s.RemovePlug),
                    out.stop:          state_name(s.Stop)
                },
                remapping={
                    'job': 'job',
                    'T_base2socket': 'T_base2socket',
                }
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
                remapping={
                    'job': 'job',
                    'T_base2socket': 'T_base2socket',
                }
            )
            StateMachine.add(
                label=state_name(s.MoveToPlugPreConnected),
                state=s.MoveToPlugPreConnected(self.config, pilot),
                transitions={
                    out.plug_pre_connected: state_name(s.InsertPlug),
                    out.stop:               state_name(s.Stop)
                },
                remapping={
                    'job': 'job',
                    'T_base2socket': 'T_base2socket',
                }
            )
            StateMachine.add(
                label=state_name(s.InsertPlug),
                state=s.InsertPlug(self.config, pilot),
                transitions={
                    out.plug_connected: state_name(s.ReleasePlug),
                    out.stop:           state_name(s.Stop)
                },
                remapping={
                    'job': 'job',
                    'T_base2socket': 'T_base2socket',
                }
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
