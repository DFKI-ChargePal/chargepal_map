from __future__ import annotations

# libs
import copy
from pathlib import Path
from smach import StateMachine, UserData

import chargepal_map.state_machine.states as s
import chargepal_map.state_machine.outcomes as out
from chargepal_map.state_machine.step_by_user import StepByUser
from chargepal_map.state_machine.arm_status import ArmStatusServer
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
        # Create step by user service if enabled
        self.step_by_user = StepByUser(self.config['step_by_user'])
        # Create arm status server
        self.arm_status = ArmStatusServer(self.config)
        self.state_machine = StateMachine(
            outcomes=[
                out.job_stopped, 
                out.job_failed, 
                out.job_incomplete, 
                out.job_complete,
            ],
            input_keys=['job', 'battery_id'])

    def execute(self, ud: UserData) -> str:
        outcome: str = self.state_machine.execute(ud)
        return outcome

    def build(self, pilot: Pilot) -> None:
        # self.arm_status.initial_check(pilot)
        with self.state_machine:
            StateMachine.add(
                label=state_name(s.ArrangeJob),
                state=s.ArrangeJob(self.config, pilot, None),
                transitions={
                    out.arm_in_wrong_ws:      state_name(s.FlipArm),
                    out.arm_ready_to_go:      state_name(s.MoveToBatteryObs),
                    out.arm_ready_to_free:    state_name(s.DriveFree),
                    out.arm_ready_to_move_ls: state_name(s.MoveToStartLs),
                    out.arm_ready_to_move_rs: state_name(s.MoveToStartRs),
                    out.job_stopped:          state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToStartLs),
                state=s.MoveToStartLs(self.config, pilot, None),
                transitions={
                    out.job_complete: state_name(s.Completion),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToStartRs),
                state=s.MoveToStartRs(self.config, pilot, None),
                transitions={
                    out.job_complete: state_name(s.Completion),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.DriveFree),
                state=s.DriveFree(self.config, pilot, None),
                transitions={
                    out.job_complete: state_name(s.Completion),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.FlipArm),
                state=s.FlipArm(self.config, pilot, self.step_by_user),
                transitions={
                    out.arm_ready_to_go: state_name(s.MoveToBatteryObs),
                    out.job_stopped:     state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToBatteryObs),
                state=s.MoveToBatteryObs(self.config, pilot, self.step_by_user),
                transitions={
                    out.battery_pre_obs: state_name(s.ObserveBattery),
                    out.job_stopped:     state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.ObserveBattery),
                state=s.ObserveBattery(self.config, pilot, None),
                transitions={
                    out.battery_obs_plug_in:  state_name(s.MoveToPlugIdObs),
                    out.battery_obs_plug_out: state_name(s.MoveToPeripheryObs),
                    out.err_obs_recover:      state_name(s.MoveToIncompletion),
                    out.job_stopped:          state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToPlugIdObs),
                state=s.MoveToPlugIdObs(self.config, pilot, self.step_by_user),
                transitions={
                    out.plug_id_pre_obs: state_name(s.ObservePlugId),
                    out.job_stopped:     state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.ObservePlugId),
                state=s.ObservePlugId(self.config, pilot, None),
                transitions={
                    out.battery_plug_id_obs:   state_name(s.MoveToPlugPrePos),
                    out.periphery_plug_id_obs: state_name(s.MoveToPlugObs),
                    out.err_obs_recover:       state_name(s.MoveToIncompletion),
                    out.job_stopped:           state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToPlugObs),
                state=s.MoveToPlugObs(self.config, pilot, self.step_by_user),
                transitions={
                    out.plug_pre_obs: state_name(s.ObservePlug),
                    out.job_stopped:  state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.ObservePlug),
                state=s.ObservePlug(self.config, pilot, None),
                transitions={
                    out.plug_obs:        state_name(s.MoveToPlugPrePos),
                    out.err_obs_recover: state_name(s.MoveToIncompletion),
                    out.job_stopped:     state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToPlugPrePos),
                state=s.MoveToPlugPrePos(self.config, pilot, self.step_by_user),
                transitions={
                    out.plug_pre_pos: state_name(s.AttachPlug),
                    out.job_stopped:  state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.AttachPlug),
                state=s.AttachPlug(self.config, pilot, self.step_by_user),
                transitions={
                    out.plug_attached:        state_name(s.RemovePlug),
                    out.err_plug_out_retry:   state_name(s.MoveToPlugObs),
                    out.err_plug_out_recover: state_name(s.MoveToIncompletion),
                    out.err_plug_out_stop:    state_name(s.Malfunction),
                    out.job_stopped:          state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.RemovePlug),
                state=s.RemovePlug(self.config, pilot, self.step_by_user),
                transitions={
                    out.battery_plug_removed:   state_name(s.MoveToPeripheryObs),
                    out.periphery_plug_removed: state_name(s.MoveToSocketPrePos),
                    out.err_plug_out_stop:      state_name(s.Malfunction),
                    out.job_stopped:            state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToPeripheryObs),
                state=s.MoveToPeripheryObs(self.config, pilot, self.step_by_user),
                transitions={
                    out.periphery_pre_obs: state_name(s.ObservePeriphery),
                    out.job_stopped:       state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.ObservePeriphery),
                state=s.ObservePeriphery(self.config, pilot, None),
                transitions={
                    out.periphery_plug_obs:   state_name(s.MoveToPlugIdObs),
                    out.periphery_socket_obs: state_name(s.MoveToSocketObs),
                    out.err_obs_retry:        state_name(s.MoveToPeripheryObs),
                    out.err_plug_in_recover:  state_name(s.MoveToRecoverPrePos),
                    out.err_plug_out_recover: state_name(s.MoveToIncompletion),
                    out.job_stopped:          state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToSocketObs),
                state=s.MoveToSocketObs(self.config, pilot, self.step_by_user),
                transitions={
                    out.socket_pre_obs: state_name(s.ObserveSocket),
                    out.job_stopped:    state_name(s.Stop)
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.ObserveSocket),
                state=s.ObserveSocket(self.config, pilot, None),
                transitions={
                    out.socket_obs:      state_name(s.MoveToSocketPrePos),
                    out.err_obs_recover: state_name(s.MoveToRecoverPrePos),
                    out.job_stopped:     state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToSocketPrePos),
                state=s.MoveToSocketPrePos(self.config, pilot, self.step_by_user),
                transitions={
                    out.socket_pre_pos: state_name(s.InsertPlug),
                    out.job_stopped:    state_name(s.Stop)
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToRecoverPrePos),
                state=s.MoveToRecoverPrePos(self.config, pilot, self.step_by_user),
                transitions={
                    out.recover_pre_pos: state_name(s.InsertPlug),
                    out.job_stopped:     state_name(s.Stop)
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.InsertPlug),
                state=s.InsertPlug(self.config, pilot, self.step_by_user),
                transitions={
                    out.plug_connected:              state_name(s.ReleasePlug),
                    out.err_plug_in_recover:         state_name(s.MoveToRecoverPrePos),
                    out.err_plug_in_battery_retry:   state_name(s.MoveToSocketPrePos),
                    out.err_plug_in_periphery_retry: state_name(s.MoveToSocketObs),
                    out.err_plug_in_stop:            state_name(s.Malfunction),
                    out.job_stopped:                 state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.ReleasePlug),
                state=s.ReleasePlug(self.config, pilot, self.step_by_user),
                transitions={
                    out.plug_released:       state_name(s.MoveToCompletion),
                    out.err_plug_in_recover: state_name(s.MoveToIncompletion),
                    out.err_plug_in_stop:    state_name(s.Malfunction),
                    out.job_stopped:         state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.MoveToCompletion),
                state=s.MoveToCompletion(self.config, pilot, self.step_by_user),
                transitions={
                    out.job_complete: state_name(s.Completion),
                    out.job_stopped:  state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.Completion),
                state=s.Completion(self.config, pilot),
                transitions={out.job_complete: out.job_complete}
            )
            StateMachine.add(
                label=state_name(s.MoveToIncompletion),
                state=s.MoveToIncompletion(self.config, pilot, self.step_by_user),
                transitions={
                    out.job_incomplete: state_name(s.Incompletion),
                    out.job_stopped:    state_name(s.Stop),
                },
                remapping={'job': 'job', 'battery_id': 'battery_id'}
            )
            StateMachine.add(
                label=state_name(s.Incompletion),
                state=s.Incompletion(self.config, pilot),
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
                transitions={out.job_stopped: out.job_stopped}
            )
