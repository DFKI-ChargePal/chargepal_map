from __future__ import annotations

# global
import rospy
import actionlib
from pathlib import Path
from smach import StateMachine

# local
import chargepal_map.state_machine.outcomes as out
import chargepal_map.state_machine.states.common as com
from chargepal_map.state_machine.process import ProcessABC
import chargepal_map.state_machine.states.connect_to_car as ctc
import chargepal_map.state_machine.states.connect_to_car_electric as ctc_e
import chargepal_map.state_machine.states.disconnect_from_car as dfc
import chargepal_map.state_machine.states.disconnect_from_car_electric as dfc_e
from chargepal_map.state_machine.utils import state_name

# actions
from chargepal_actions.msg import (
    ConnectPlugToCarAction, 
    ConnectPlugToCarActionGoal,
    ConnectPlugToCarActionFeedback,
    DisconnectPlugFromCarAction,
    DisconnectPlugFromCarActionGoal,
    DisconnectPlugFromCarFeedback,
)

# typing
from typing import Type
from ur_pilot import Pilot


class ConnectToCar(ProcessABC):

    def __init__(self, name: str, cfg_fp: Path, dtt_dir: Path) -> None:
        super().__init__(name, cfg_fp, dtt_dir)
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

    def wait_for_usr_feedback(self) -> None:
        feedback = ConnectPlugToCarActionFeedback()
        feedback.status = "wait_for_user"
        self.action_server.publish_feedback(feedback)

    def set_up(self, pilot: Pilot) -> None:
        # Open smash container to add states and transitions
        with self.state_machine:
            StateMachine.add(
                label=state_name(ctc.MoveArmToBattery),
                state=ctc.MoveArmToBattery(self.config, pilot),
                transitions={out.ConnectToCar.arm_in_bat_obs: state_name(ctc.ObservePlugOnBattery),
                             out.Common.stop:                 state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc.ObservePlugOnBattery),
                state=ctc.ObservePlugOnBattery(self.config, pilot),
                transitions={out.ConnectToCar.arm_in_bat_pre_connect: state_name(ctc.GraspPlugOnBattery),
                             out.Common.stop:                         state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc.GraspPlugOnBattery),
                state=ctc.GraspPlugOnBattery(self.config, pilot),
                transitions={out.ConnectToCar.plug_in_bat_connect: state_name(ctc.RemovePlugFromBattery),
                             out.Common.stop:                      state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc.RemovePlugFromBattery),
                state=ctc.RemovePlugFromBattery(self.config, pilot),
                transitions={out.ConnectToCar.plug_in_bat_post_connect: state_name(ctc.MovePlugToCar),
                             out.Common.stop:                           state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc.MovePlugToCar),
                state=ctc.MovePlugToCar(self.config, pilot),
                transitions={out.ConnectToCar.plug_in_car_obs: state_name(ctc.ObserveSocketOnCar),
                             out.Common.stop:                  state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc.ObserveSocketOnCar),
                state=ctc.ObserveSocketOnCar(self.config, pilot),
                transitions={out.ConnectToCar.plug_in_car_pre_connect: state_name(ctc.InsertPlugToCar),
                             out.Common.stop:                          state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc.InsertPlugToCar),
                state=ctc.InsertPlugToCar(self.config, pilot),
                transitions={out.ConnectToCar.plug_in_car_connect: state_name(ctc.ReleasePlugOnCar),
                             out.Common.stop:                      state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc.ReleasePlugOnCar),
                state=ctc.ReleasePlugOnCar(self.config, pilot),
                transitions={out.ConnectToCar.arm_in_car_post_connect: state_name(ctc.MoveArmToDrivePos),
                             out.Common.stop:                          state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc.MoveArmToDrivePos),
                state=ctc.MoveArmToDrivePos(self.config, pilot),
                transitions={out.ConnectToCar.arm_in_driving_pose: out.ConnectToCar.arm_in_driving_pose}
            )
            StateMachine.add(
                label=state_name(com.Stop),
                state=com.Stop(self.config, pilot),
                transitions={out.Common.stop: out.Common.stop}
            )


class ConnectToCarElectric(ConnectToCar):

    def __init__(self, name: str, cfg_fp: Path, dtt_dir: Path) -> None:
        super().__init__('connect_to_car', cfg_fp, dtt_dir)


    def set_up(self, pilot: Pilot) -> None:
        # Open smash container to add states and transitions
        with self.state_machine:
            StateMachine.add(
                label=state_name(ctc_e.MoveArmToBattery),
                state=ctc_e.MoveArmToBattery(self.config, pilot),
                transitions={out.ConnectToCarElectric.arm_in_bat_pre_obs: state_name(ctc_e.ObservePlugOnBattery),
                             out.Common.stop:                             state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_e.ObservePlugOnBattery),
                state=ctc_e.ObservePlugOnBattery(self.config, pilot),
                transitions={out.ConnectToCarElectric.arm_in_bat_post_obs: state_name(ctc_e.MoveArmToBatteryPreGrasp),
                             out.Common.stop:                              state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc_e.MoveArmToBatteryPreGrasp),
                state=ctc_e.MoveArmToBatteryPreGrasp(self.config, pilot),
                transitions={out.ConnectToCarElectric.arm_in_bat_pre_connect: state_name(ctc_e.GraspPlugOnBattery),
                             out.Common.stop:                                 state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc_e.GraspPlugOnBattery),
                state=ctc_e.GraspPlugOnBattery(self.config, pilot),
                transitions={out.ConnectToCarElectric.plug_in_bat_connect: state_name(ctc_e.RemovePlugFromBattery),
                             out.Common.stop:                              state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_e.RemovePlugFromBattery),
                state=ctc_e.RemovePlugFromBattery(self.config, pilot),
                transitions={out.ConnectToCarElectric.plug_in_bat_post_connect: state_name(ctc_e.MovePlugToCar),
                             out.Common.stop:                                   state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_e.MovePlugToCar),
                state=ctc_e.MovePlugToCar(self.config, pilot),
                transitions={out.ConnectToCarElectric.plug_in_car_pre_obs: state_name(ctc_e.ObserveSocketOnCar),
                             out.Common.stop:                              state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_e.ObserveSocketOnCar),
                state=ctc_e.ObserveSocketOnCar(self.config, pilot),
                transitions={out.ConnectToCarElectric.plug_in_car_post_obs: state_name(ctc_e.MovePlugToCarPreConnect),
                             out.Common.stop:                               state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc_e.MovePlugToCarPreConnect),
                state=ctc_e.MovePlugToCarPreConnect(self.config, pilot),
                transitions={out.ConnectToCarElectric.plug_in_car_pre_connect: state_name(ctc_e.InsertPlugToCar),
                             out.Common.stop:                                  state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc_e.InsertPlugToCar),
                state=ctc_e.InsertPlugToCar(self.config, pilot),
                transitions={out.ConnectToCarElectric.plug_in_car_connect: state_name(ctc_e.ReleasePlugOnCar),
                             out.Common.stop:                              state_name(com.Stop)}
            )

            StateMachine.add(
                label=state_name(ctc_e.ReleasePlugOnCar),
                state=ctc_e.ReleasePlugOnCar(self.config, pilot),
                transitions={out.ConnectToCarElectric.arm_in_car_post_connect: state_name(ctc_e.MoveArmToDrivePos),
                             out.Common.stop:                                  state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_e.MoveArmToDrivePos),
                state=ctc_e.MoveArmToDrivePos(self.config, pilot),
                transitions={out.ConnectToCarElectric.arm_in_driving_pose: out.ConnectToCarElectric.arm_in_driving_pose}
            )
            StateMachine.add(
                label=state_name(com.Stop),
                state=com.Stop(self.config, pilot),
                transitions={out.Common.stop: out.Common.stop}
            )


class DisconnectFromCar(ProcessABC):

    def __init__(self, name: str, cfg_fp: Path, dtt_dir: Path) -> None:
        super().__init__(name, cfg_fp, dtt_dir)
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

    def wait_for_usr_feedback(self) -> None:
        feedback = DisconnectPlugFromCarFeedback()
        feedback.status = "wait_for_user"
        self.action_server.publish_feedback(feedback)

    def set_up(self, pilot: Pilot) -> None:
        # Open smash container to add states and transitions
        with self.state_machine:
            StateMachine.add(
                label=state_name(dfc.MoveArmToCar),
                state=dfc.MoveArmToCar(self.config, pilot),
                transitions={out.DisconnectFromCar.arm_in_car_obs: state_name(dfc.ObservePlugOnCar),
                             out.Common.stop:                      state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc.ObservePlugOnCar),
                state=dfc.ObservePlugOnCar(self.config, pilot),
                transitions={out.DisconnectFromCar.arm_in_car_pre_connect: state_name(dfc.GraspPlugOnCar),
                             out.Common.stop:                              state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc.GraspPlugOnCar),
                state=dfc.GraspPlugOnCar(self.config, pilot),
                transitions={out.DisconnectFromCar.plug_in_car_connect: state_name(dfc.RemovePlugFromCar),
                             out.Common.stop:                           state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc.RemovePlugFromCar),
                state=dfc.RemovePlugFromCar(self.config, pilot),
                transitions={out.DisconnectFromCar.plug_in_car_post_connect: state_name(dfc.MovePlugToBattery),
                             out.Common.stop:                                state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc.MovePlugToBattery),
                state=dfc.MovePlugToBattery(self.config, pilot),
                transitions={out.DisconnectFromCar.plug_in_bat_obs: state_name(dfc.ObserveSocketOnBattery),
                             out.Common.stop:                       state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc.ObserveSocketOnBattery),
                state=dfc.ObserveSocketOnBattery(self.config, pilot),
                transitions={out.DisconnectFromCar.plug_in_bat_pre_connect: state_name(dfc.InsertPlugToBattery),
                             out.Common.stop:                               state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc.InsertPlugToBattery),
                state=dfc.InsertPlugToBattery(self.config, pilot),
                transitions={out.DisconnectFromCar.plug_in_bat_connect: state_name(dfc.ReleasePlugOnBattery),
                             out.Common.stop:                           state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc.ReleasePlugOnBattery),
                state=dfc.ReleasePlugOnBattery(self.config, pilot),
                transitions={out.DisconnectFromCar.arm_in_bat_post_connect: state_name(dfc.MoveArmToDrivePos),
                             out.Common.stop:                               state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc.MoveArmToDrivePos),
                state=dfc.MoveArmToDrivePos(self.config, pilot),
                transitions={out.DisconnectFromCar.arm_in_driving_pose: out.DisconnectFromCar.arm_in_driving_pose}
            )
            StateMachine.add(
                label=state_name(com.Stop),
                state=com.Stop(self.config, pilot),
                transitions={out.Common.stop: out.Common.stop}
            )


class DisconnectFromCarElectric(DisconnectFromCar):

    def __init__(self, name: str, cfg_fp: Path, dtt_dir: Path) -> None:
        super().__init__('disconnect_from_car', cfg_fp, dtt_dir)

    def set_up(self, pilot: Pilot) -> None:
        # Open smach container to add states and transitions
        with self.state_machine:
            StateMachine.add(
                label=state_name(dfc_e.MoveArmToCar),
                state=dfc_e.MoveArmToCar(self.config, pilot),
                transitions={out.DisconnectFromCarElectric.arm_in_car_pre_obs: state_name(dfc_e.ObservePlugOnCar),
                             out.Common.stop:                                  state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_e.ObservePlugOnCar),
                state=dfc_e.ObservePlugOnCar(self.config, pilot),
                transitions={out.DisconnectFromCarElectric.arm_in_car_post_obs: state_name(dfc_e.MoveArmToCarPreGrasp),
                             out.Common.stop:                                   state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc_e.MoveArmToCarPreGrasp),
                state=dfc_e.MoveArmToCarPreGrasp(self.config, pilot),
                transitions={out.DisconnectFromCarElectric.arm_in_car_pre_connect: state_name(dfc_e.GraspPlugOnCar),
                             out.Common.stop:                                      state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc_e.GraspPlugOnCar),
                state=dfc_e.GraspPlugOnCar(self.config, pilot),
                transitions={out.DisconnectFromCarElectric.plug_in_car_connect: state_name(dfc_e.RemovePlugFromCar),
                             out.Common.stop:                                   state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_e.RemovePlugFromCar),
                state=dfc_e.RemovePlugFromCar(self.config, pilot),
                transitions={out.DisconnectFromCarElectric.plug_in_car_post_connect: state_name(dfc_e.MovePlugToBattery),
                             out.Common.stop:                                        state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_e.MovePlugToBattery),
                state=dfc_e.MovePlugToBattery(self.config, pilot),
                transitions={out.DisconnectFromCarElectric.plug_in_bat_pre_obs: state_name(dfc_e.ObserveSocketOnBattery),
                             out.Common.stop:                                   state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_e.ObserveSocketOnBattery),
                state=dfc_e.ObserveSocketOnBattery(self.config, pilot),
                transitions={out.DisconnectFromCarElectric.plug_in_bat_post_obs: state_name(dfc_e.MovePlugToBatteryPreConnect),
                             out.Common.stop:                                    state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc_e.MovePlugToBatteryPreConnect),
                state=dfc_e.MovePlugToBatteryPreConnect(self.config, pilot),
                transitions={out.DisconnectFromCarElectric.plug_in_bat_pre_connect: state_name(dfc_e.InsertPlugToBattery),
                             out.Common.stop:                                       state_name(com.Stop)},
                remapping={'xyz_xyzw_base2socket': 'xyz_xyzw_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc_e.InsertPlugToBattery),
                state=dfc_e.InsertPlugToBattery(self.config, pilot),
                transitions={out.DisconnectFromCarElectric.plug_in_bat_connect: state_name(dfc_e.ReleasePlugOnBattery),
                             out.Common.stop:                                   state_name(com.Stop)}
            )

            StateMachine.add(
                label=state_name(dfc_e.ReleasePlugOnBattery),
                state=dfc_e.ReleasePlugOnBattery(self.config, pilot),
                transitions={out.DisconnectFromCarElectric.arm_in_bat_post_connect: state_name(dfc_e.MoveArmToDrivePos),
                             out.Common.stop:                                       state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_e.MoveArmToDrivePos),
                state=dfc_e.MoveArmToDrivePos(self.config, pilot),
                transitions={out.DisconnectFromCarElectric.arm_in_driving_pose: out.DisconnectFromCarElectric.arm_in_driving_pose}
            )
            StateMachine.add(
                label=state_name(com.Stop),
                state=com.Stop(self.config, pilot),
                transitions={out.Common.stop: out.Common.stop}
            )


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
manipulation_action_processor.register('connect_to_car', ConnectToCar)
manipulation_action_processor.register('connect_to_car_electric', ConnectToCarElectric)
manipulation_action_processor.register('disconnect_from_car', DisconnectFromCar)
manipulation_action_processor.register('disconnect_from_car_electric', DisconnectFromCarElectric)
