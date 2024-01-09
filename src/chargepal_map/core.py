from __future__ import annotations

# global
import abc
import yaml
import rospy
import actionlib
from pathlib import Path
from smach import StateMachine

# local
import chargepal_map.state_machine.outcomes as out
import chargepal_map.state_machine.states.common as com
import chargepal_map.state_machine.states.connect_to_car as ctc
import chargepal_map.state_machine.states.disconnect_from_car as dfc
from chargepal_map.state_machine.utils import (
    state_name, 
    silent_smach
)

# actions
from chargepal_actions.msg import (
    ConnectPlugToCarAction, 
    ConnectPlugToCarActionGoal,
    ConnectPlugToCarActionFeedback,
    DisconnectPlugFromCarAction,
    DisconnectPlugFromCarActionGoal,
)

# typing 
from typing import Any, Type


class StateMachineBuilder(metaclass=abc.ABCMeta):

    def __init__(self, name: str, cfg_fp: Path):
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

    @abc.abstractmethod
    def action_callback(self, goal: Any) -> None:
        raise NotImplementedError("Must be implemented in child class")

    @abc.abstractmethod
    def set_up(self) -> None:
        raise NotImplementedError("Must be implemented in child class")


class ConnectToCar(StateMachineBuilder):

    def __init__(self, name: str, cfg_fp: Path) -> None:
        super().__init__(name, cfg_fp)
        self.state_machine = StateMachine(outcomes=[out.Common.stop, out.ConnectToCar.arm_in_driving_pose])
        self.action_server = actionlib.SimpleActionServer(self.name, 
                                                          ConnectPlugToCarAction, self.action_callback, False)
        self.action_server.start()

    def action_callback(self, goal: ConnectPlugToCarActionGoal) -> None:
        rospy.loginfo(f"Approach connect plug to car process")
        try:
            rospy.loginfo(f"Process connect task step by step")
            # Execute SMACH plan
            outcome = self.state_machine.execute()
            self.action_server.set_succeeded()
            rospy.loginfo(f"Finish connect process successfully.")
        except Exception as e:
            rospy.logwarn(f"Error while plugging process: {e}")
            self.action_server.set_aborted()
        rospy.loginfo(f"Leaving connect plug to car process")

    def action_feedback(self) -> None:
        feedback = ConnectPlugToCarActionFeedback()
        feedback.status = "BlaBlaBla"
        self.action_server.publish_feedback(feedback)

    def set_up(self) -> None:
        # Open smash container to add states and transitions
        with self.state_machine:
            StateMachine.add(
                label=state_name(ctc.MoveArmToBattery),
                state=ctc.MoveArmToBattery(self.config, self.action_server),
                transitions={out.ConnectToCar.arm_in_bat_obs: state_name(ctc.ObservePlugOnBattery),
                             out.Common.stop:                 state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc.ObservePlugOnBattery),
                state=ctc.ObservePlugOnBattery(self.config, self.action_server),
                transitions={out.ConnectToCar.arm_in_bat_pre_connect: state_name(ctc.GraspPlugOnBattery),
                             out.Common.stop:                         state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc.GraspPlugOnBattery),
                state=ctc.GraspPlugOnBattery(self.config, self.action_server),
                transitions={out.ConnectToCar.plug_in_bat_connect: state_name(ctc.RemovePlugFromBattery),
                             out.Common.stop:                      state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc.RemovePlugFromBattery),
                state=ctc.RemovePlugFromBattery(self.config, self.action_server),
                transitions={out.ConnectToCar.plug_in_bat_post_connect: state_name(ctc.MovePlugToCar),
                             out.Common.stop:                           state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc.MovePlugToCar),
                state=ctc.MovePlugToCar(self.config, self.action_server),
                transitions={out.ConnectToCar.plug_in_car_obs: state_name(ctc.ObserveSocketOnCar),
                             out.Common.stop:                  state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc.ObserveSocketOnCar),
                state=ctc.ObserveSocketOnCar(self.config, self.action_server),
                transitions={out.ConnectToCar.plug_in_car_pre_connect: state_name(ctc.InsertPlugToCar),
                             out.Common.stop:                          state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc.InsertPlugToCar),
                state=ctc.InsertPlugToCar(self.config, self.action_server),
                transitions={out.ConnectToCar.plug_in_car_connect: state_name(ctc.ReleasePlugOnCar),
                             out.Common.stop:                      state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc.ReleasePlugOnCar),
                state=ctc.ReleasePlugOnCar(self.config, self.action_server),
                transitions={out.ConnectToCar.arm_in_car_post_connect: state_name(ctc.MoveArmToDrivePos),
                             out.Common.stop:                          state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc.MoveArmToDrivePos),
                state=ctc.MoveArmToDrivePos(self.config, self.action_server),
                transitions={out.ConnectToCar.arm_in_driving_pose: out.ConnectToCar.arm_in_driving_pose}
            )
            StateMachine.add(
                label=state_name(com.Stop),
                state=com.Stop(self.config, self.action_server),
                transitions={out.Common.stop: out.Common.stop}
            )


class DisconnectFromCar(StateMachineBuilder):

    def __init__(self, name: str, cfg_fp: Path) -> None:
        super().__init__(name, cfg_fp)
        self.state_machine = StateMachine(outcomes=[out.Common.stop, out.DisconnectFromCar.arm_in_driving_pose])
        self.action_server = actionlib.SimpleActionServer(self.name, 
                                                          DisconnectPlugFromCarAction, self.action_callback, False)
        self.action_server.start()

    def action_callback(self, goal: DisconnectPlugFromCarActionGoal) -> None:
        rospy.loginfo(f"Approach disconnect plug from car process")
        try:
            rospy.loginfo(f"Process disconnect task step by step")
            # Execute SMACH plan
            outcome = self.state_machine.execute()
            self.action_server.set_succeeded()
            rospy.loginfo(f"Finish disconnect process successfully.")
        except Exception as e:
            rospy.logwarn(f"Error while plugging process: {e}")
            self.action_server.set_aborted()
        rospy.loginfo(f"Leaving disconnect plug from car process")

    def set_up(self) -> None:
        # Open smash container to add states and transitions
        with self.state_machine:
            StateMachine.add(
                label=state_name(dfc.MoveArmToCar),
                state=dfc.MoveArmToCar(self.config, self.action_server),
                transitions={out.DisconnectFromCar.arm_in_car_obs: state_name(dfc.ObservePlugOnCar),
                             out.Common.stop:                      state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc.ObservePlugOnCar),
                state=dfc.ObservePlugOnCar(self.config, self.action_server),
                transitions={out.DisconnectFromCar.arm_in_car_pre_connect: state_name(dfc.GraspPlugOnCar),
                             out.Common.stop:                              state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc.GraspPlugOnCar),
                state=dfc.GraspPlugOnCar(self.config, self.action_server),
                transitions={out.DisconnectFromCar.plug_in_car_connect: state_name(dfc.RemovePlugFromCar),
                             out.Common.stop:                           state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc.RemovePlugFromCar),
                state=dfc.RemovePlugFromCar(self.config, self.action_server),
                transitions={out.DisconnectFromCar.plug_in_car_post_connect: state_name(dfc.MovePlugToBattery),
                             out.Common.stop:                                state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc.MovePlugToBattery),
                state=dfc.MovePlugToBattery(self.config, self.action_server),
                transitions={out.DisconnectFromCar.plug_in_bat_obs: state_name(dfc.ObserveSocketOnBattery),
                             out.Common.stop:                       state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc.ObserveSocketOnBattery),
                state=dfc.ObserveSocketOnBattery(self.config, self.action_server),
                transitions={out.DisconnectFromCar.plug_in_bat_pre_connect: state_name(dfc.InsertPlugToBattery),
                             out.Common.stop:                               state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc.InsertPlugToBattery),
                state=dfc.InsertPlugToBattery(self.config, self.action_server),
                transitions={out.DisconnectFromCar.plug_in_bat_connect: state_name(dfc.ReleasePlugOnBattery),
                             out.Common.stop:                           state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc.ReleasePlugOnBattery),
                state=dfc.ReleasePlugOnBattery(self.config, self.action_server),
                transitions={out.DisconnectFromCar.arm_in_bat_post_connect: state_name(dfc.MoveArmToDrivePos),
                             out.Common.stop:                               state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc.MoveArmToDrivePos),
                state=dfc.MoveArmToDrivePos(self.config, self.action_server),
                transitions={out.DisconnectFromCar.arm_in_driving_pose: out.DisconnectFromCar.arm_in_driving_pose}
            )
            StateMachine.add(
                label=state_name(com.Stop),
                state=com.Stop(self.config, self.action_server),
                transitions={out.Common.stop: out.Common.stop}
            )


class ProcessFactory:

    def __init__(self) -> None:
        self._selection: dict[str, Type[StateMachineBuilder]] = {}

    def register_process(self, name: str, process: Type[StateMachineBuilder]) -> None:
        self._selection[name] = process

    def create(self, name: str, cfg_dir: Path) -> StateMachineBuilder:
        cfg_fp = cfg_dir.joinpath(name + '.yaml')
        builder = self._selection[name](name=name, cfg_fp=cfg_fp)
        builder.set_up()
        return builder


manipulation_action_processor = ProcessFactory()
manipulation_action_processor.register_process('connect_to_car', ConnectToCar)
manipulation_action_processor.register_process('disconnect_from_car', DisconnectFromCar)
