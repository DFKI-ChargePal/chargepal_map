from __future__ import annotations

# libs
import copy
from pathlib import Path
from smach import StateMachine, UserData

import chargepal_map.state_machine.states as s
import chargepal_map.state_machine.outcomes as out
from chargepal_map.state_machine.step_by_user import StepByUser
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
        self.step_by_user = StepByUser(self.config['step_by_user'])
        self.state_machine = StateMachine(
            outcomes=[
                out.job_stopped, 
                out.job_failed, 
                out.job_incomplete, 
                out.completed
            ],
            input_keys=['job_id'])

    def execute(self, ud: UserData) -> str:
        return self.state_machine.execute(ud)

    def build(self, pilot: Pilot) -> None:
        with self.state_machine:
            StateMachine.add(
                label=state_name(s.ArrangeJob),
                state=s.ArrangeJob(self.config, pilot, None),
                transitions={
                    out.arm_in_wrong_ws:      state_name(s.FlipArm),
                    out.arm_ready_to_go:      state_name(s.MoveToSceneObs),
                    out.arm_ready_to_free:    state_name(s.DriveFree),
                    out.arm_ready_to_move_ls: state_name(s.MoveToStartRs),
                    out.arm_ready_to_move_rs: state_name(s.MoveToStartLs),
                    out.job_stopped:          state_name(s.Stop),
                },
                remapping={'job_id': 'job_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToStartLs),
                state=s.MoveToStartLs(self.config, pilot, None),
                transitions={
                    out.completed:   state_name(s.Completion),
                    out.job_stopped: state_name(s.Stop),
                }
            )
            StateMachine.add(
                label=state_name(s.MoveToStartRs),
                state=s.MoveToStartRs(self.config, pilot, None),
                transitions={
                    out.completed:   state_name(s.Completion),
                    out.job_stopped: state_name(s.Stop),
                }
            )
            StateMachine.add(
                label=state_name(s.DriveFree),
                state=s.DriveFree(self.config, pilot, None),
                transitions={
                    out.completed:   state_name(s.Completion),
                    out.job_stopped: state_name(s.Stop),
                }
            )
            StateMachine.add(
                label=state_name(s.FlipArm),
                state=s.FlipArm(self.config, pilot, self.step_by_user),
                transitions={
                    out.arm_ready_to_go: state_name(s.MoveToSceneObs),
                    out.job_stopped:     state_name(s.Stop),
                },
                remapping={'job_id': 'job_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToSceneObs),
                state=s.MoveToSceneObs(self.config, pilot, self.step_by_user),
                transitions={
                    out.arm_in_scene: state_name(s.ObserveScene),
                    out.job_stopped:  state_name(s.Stop),
                },
                remapping={'job_id': 'job_id'}
            )
            StateMachine.add(
                label=state_name(s.ObserveScene),
                state=s.ObserveScene(self.config, pilot, self.step_by_user),
                transitions={
                    out.arm_ready_to_plug_out: state_name(s.MoveToPlugObs),
                    out.arm_ready_to_plug_in:  state_name(s.MoveToSocketObs),
                    out.arm_ready_to_go:       state_name(s.MoveToSceneObs),
                    out.err_scene_incomplete:  state_name(s.MoveToIncompletion),
                    out.job_stopped:           state_name(s.Stop),
                },
                remapping={
                    'job_id': 'job_id',
                    'T_base2socket': 'T_base2socket',
                }
            )
            StateMachine.add(
                label=state_name(s.MoveToPlugObs),
                state=s.MoveToPlugObs(self.config, pilot, self.step_by_user),
                transitions={
                    out.plug_pre_obs: state_name(s.ObservePlug),
                    out.job_stopped:  state_name(s.Stop),
                },
                remapping={'job_id': 'job_id'}
            )
            StateMachine.add(
                label=state_name(s.ObservePlug),
                state=s.ObservePlug(self.config, pilot, self.step_by_user),
                transitions={
                    out.stop:                 state_name(s.Stop),
                    out.plug_obs:             state_name(s.MoveToPlugPrePos),
                    out.err_scene_incomplete: state_name(s.MoveToIncompletion),
                },
                remapping={
                    'job_id': 'job_id',
                    'T_base2socket': 'T_base2socket',
                }
            )
            StateMachine.add(
                label=state_name(s.MoveToPlugPrePos),
                state=s.MoveToPlugPrePos(self.config, pilot, self.step_by_user),
                transitions={
                    out.arm_ready_to_attach: state_name(s.AttachPlug),
                    out.job_stopped:         state_name(s.Stop),
                },
                remapping={
                    'job_id': 'job_id',
                    'T_base2socket': 'T_base2socket',
                }
            )
            StateMachine.add(
                label=state_name(s.AttachPlug),
                state=s.AttachPlug(self.config, pilot, self.step_by_user),
                transitions={
                    out.plug_attached:    state_name(s.RemovePlug),
                    out.err_arm_free:     state_name(s.MoveToPlugObs),
                    out.err_arm_not_free: state_name(s.Malfunction),
                    out.job_stopped:      state_name(s.Stop),
                },
                remapping={
                    'job_id': 'job_id',
                    'T_base2socket': 'T_base2socket',
                }
            )
            StateMachine.add(
                label=state_name(s.RemovePlug),
                state=s.RemovePlug(self.config, pilot, self.step_by_user),
                transitions={
                    out.plug_removed:            state_name(s.MoveToSceneObs),
                    out.plug_removed_completion: state_name(s.MoveToCompletion),
                    out.err_plug_not_free:       state_name(s.Malfunction),
                    out.job_stopped:             state_name(s.Stop),
                },
                remapping={'job_id': 'job_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToSocketObs),
                state=s.MoveToSocketObs(self.config, pilot, self.step_by_user),
                transitions={
                    out.socket_pre_obs: state_name(s.ObserveSocket),
                    out.job_stopped:    state_name(s.Stop)
                },
                remapping={'job_id': 'job_id'}
            )
            StateMachine.add(
                label=state_name(s.ObserveSocket),
                state=s.ObserveSocket(self.config, pilot, self.step_by_user),
                transitions={
                    out.socket_obs:           state_name(s.MoveToSocketPrePos),
                    out.err_scene_incomplete: state_name(s.MoveToIncompletion),
                    out.job_stopped:          state_name(s.Stop),
                },
                remapping={
                    'job_id': 'job_id',
                    'T_base2socket': 'T_base2socket',
                }
            )
            StateMachine.add(
                label=state_name(s.MoveToSocketPrePos),
                state=s.MoveToSocketPrePos(self.config, pilot, self.step_by_user),
                transitions={
                    out.arm_ready_to_insert: state_name(s.InsertPlug),
                    out.job_stopped:         state_name(s.Stop)
                },
                remapping={
                    'job_id': 'job_id',
                    'T_base2socket': 'T_base2socket',
                }
            )
            StateMachine.add(
                label=state_name(s.InsertPlug),
                state=s.InsertPlug(self.config, pilot, self.step_by_user),
                transitions={
                    out.plug_connected:    state_name(s.ReleasePlug),
                    out.err_plug_free:     state_name(s.MoveToSocketObs),
                    out.err_plug_not_free: state_name(s.Malfunction),
                    out.stop:              state_name(s.Stop),
                },
                remapping={
                    'job_id': 'job_id',
                    'T_base2socket': 'T_base2socket',
                }
            )
            StateMachine.add(
                label=state_name(s.ReleasePlug),
                state=s.ReleasePlug(self.config, pilot, self.step_by_user),
                transitions={
                    out.plug_released:    state_name(s.MoveToCompletion),
                    out.err_arm_not_free: state_name(s.Malfunction),
                    out.job_stopped:      state_name(s.Stop),
                },
                remapping={'job_id': 'job_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToCompletion),
                state=s.MoveToCompletion(self.config, pilot, self.step_by_user),
                transitions={
                    out.job_complete: state_name(s.Completion),
                },
            )
            StateMachine.add(
                label=state_name(s.Completion),
                state=s.Completion(self.config, pilot),
                transitions={out.completed: out.completed}
            )
            StateMachine.add(
                label=state_name(s.MoveToIncompletion),
                state=s.MoveToIncompletion(self.config, pilot, self.step_by_user),
                transitions={
                    out.job_incomplete: state_name(s.Incompletion),
                    out.job_stopped:    state_name(s.Stop),
                }
            )
            StateMachine.add(
                label=state_name(s.Incompletion),
                state=s.Completion(self.config, pilot),
                transitions={out.job_incomplete: out.job_incomplete}
            )
            StateMachine.add(
                label=state_name(s.Malfunction),
                state=s.Malfunction(self.config, pilot=pilot),
                transitions={out.job_failed: out.job_failed}
            )
            StateMachine.add(
                label=state_name(s.Stop),
                state=s.Stop(self.config, pilot),
                transitions={out.stop: out.stop}
            )
