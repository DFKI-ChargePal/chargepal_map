from __future__ import annotations

# global
import rospy
import actionlib
from pathlib import Path
from smach import StateMachine

# local
import chargepal_map.state_machine.outcomes as out
import chargepal_map.state_machine.states.common as com
from chargepal_map.state_machine.utils import state_name
from chargepal_map.state_machine.processes.process import ProcessABC
import chargepal_map.state_machine.states.disconnect_from_car_twist as dfc_t
import chargepal.chargepal_map.src.chargepal_map.state_machine.states.disconnect_from_car_twist as dfc_te
import chargepal_map.state_machine.states.disconnect_from_car_electric as dfc_e

# actions
from chargepal_actions.msg import (
    DisconnectPlugFromCarGoal,
    DisconnectPlugFromCarAction,
    DisconnectPlugFromCarResult,
    DisconnectPlugFromCarFeedback,
)

# typing
from ur_pilot import Pilot


class DisconnectFromCar(ProcessABC): 

    def __init__(self, name: str, cfg_fp: Path, dtt_dir: Path) -> None:
        super().__init__('disconnect_from_car', cfg_fp, dtt_dir)
        self.state_machine = StateMachine(outcomes=[out.Common.stop, out.DisconnectFromCarTwist.arm_in_driving_pose])
        self.action_server = actionlib.SimpleActionServer(self.name, 
                                                          DisconnectPlugFromCarAction, self.action_callback, False)
        self.action_server.start()

    def action_callback(self, goal: DisconnectPlugFromCarGoal) -> None:
        rospy.loginfo(f"Approach disconnect plug from car process")
        res_msg = DisconnectPlugFromCarResult()
        try:
            rospy.loginfo(f"Process disconnect task step by step")
            # Execute SMACH plan
            outcome = self.state_machine.execute()
            res_msg.disconnect_from_car = True
            self.action_server.set_succeeded(res_msg)
            rospy.loginfo(f"Finish disconnect process successfully.")
        except Exception as e:
            rospy.logwarn(f"Error while plugging process: {e}")
            res_msg.disconnect_from_car = False
            self.action_server.set_aborted(res_msg)
        rospy.loginfo(f"Leaving disconnect plug from car process")

    def wait_for_usr_feedback(self) -> None:
        feedback = DisconnectPlugFromCarFeedback()
        feedback.status = "wait_for_user"
        self.action_server.publish_feedback(feedback)

    def set_up(self, pilot: Pilot) -> None:
        raise NotImplementedError('Must be implemented in subclass')


class DisconnectFromCarTwist(DisconnectFromCar):

    def __init__(self, name: str, cfg_fp: Path, dtt_dir: Path) -> None:
        super().__init__(name, cfg_fp, dtt_dir)

    def set_up(self, pilot: Pilot) -> None:
        # Open smash container to add states and transitions
        with self.state_machine:
            StateMachine.add(
                label=state_name(dfc_t.MoveArmToCar),
                state=dfc_t.MoveArmToCar(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.arm_in_car_pre_obs: state_name(dfc_t.ObservePlugOnCar),
                             out.Common.stop:                               state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_t.ObservePlugOnCar),
                state=dfc_t.ObservePlugOnCar(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.arm_in_car_post_obs: state_name(dfc_t.MoveArmToCarPreGrasp),
                             out.Common.stop:                                state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc_t.MoveArmToCarPreGrasp),
                state=dfc_t.MoveArmToCarPreGrasp(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.arm_in_car_pre_connect: state_name(dfc_t.GraspPlugOnCar),
                             out.Common.stop:                                   state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc_t.GraspPlugOnCar),
                state=dfc_t.GraspPlugOnCar(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.plug_in_car_connect: state_name(dfc_t.RemovePlugFromCar),
                             out.Common.stop:                                state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_t.RemovePlugFromCar),
                state=dfc_t.RemovePlugFromCar(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.plug_in_car_post_connect: state_name(dfc_t.MovePlugToBattery),
                             out.Common.stop:                                state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_t.MovePlugToBattery),
                state=dfc_t.MovePlugToBattery(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.plug_in_bat_pre_obs: state_name(dfc_t.ObserveSocketOnBattery),
                             out.Common.stop:                       state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_t.ObserveSocketOnBattery),
                state=dfc_t.ObserveSocketOnBattery(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.plug_in_bat_post_obs: state_name(dfc_t.MovePlugToBatteryPreConnect),
                             out.Common.stop:                                 state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc_t.MovePlugToBatteryPreConnect),
                state=dfc_t.MovePlugToBatteryPreConnect(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.plug_in_bat_pre_connect: state_name(dfc_t.InsertPlugToBattery),
                             out.Common.stop:                                    state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc_t.InsertPlugToBattery),
                state=dfc_t.InsertPlugToBattery(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.plug_in_bat_connect: state_name(dfc_t.ReleasePlugOnBattery),
                             out.Common.stop:                                state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_t.ReleasePlugOnBattery),
                state=dfc_t.ReleasePlugOnBattery(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.arm_in_bat_post_connect: state_name(dfc_t.MoveArmToDrivePos),
                             out.Common.stop:                                    state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_t.MoveArmToDrivePos),
                state=dfc_t.MoveArmToDrivePos(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.arm_in_driving_pose: out.DisconnectFromCarTwist.arm_in_driving_pose}
            )
            StateMachine.add(
                label=state_name(com.Stop),
                state=com.Stop(self.config, pilot),
                transitions={out.Common.stop: out.Common.stop}
            )


class DisconnectFromCarTwistEasy(DisconnectFromCar):

    def __init__(self, name: str, cfg_fp: Path, dtt_dir: Path) -> None:
        super().__init__(name, cfg_fp, dtt_dir)

    def set_up(self, pilot: Pilot) -> None:
        # Open smash container to add states and transitions
        with self.state_machine:
            StateMachine.add(
                label=state_name(dfc_te.MoveArmToCar),
                state=dfc_te.MoveArmToCar(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.arm_in_car_pre_obs: state_name(dfc_te.ObservePlugOnCar),
                             out.Common.stop:                               state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_te.ObservePlugOnCar),
                state=dfc_te.ObservePlugOnCar(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.arm_in_car_post_obs: state_name(dfc_te.MoveArmToCarPreGrasp),
                             out.Common.stop:                                state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc_te.MoveArmToCarPreGrasp),
                state=dfc_te.MoveArmToCarPreGrasp(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.arm_in_car_pre_connect: state_name(dfc_te.GraspPlugOnCar),
                             out.Common.stop:                                   state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc_te.GraspPlugOnCar),
                state=dfc_te.GraspPlugOnCar(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.plug_in_car_connect: state_name(dfc_te.RemovePlugFromCar),
                             out.Common.stop:                                state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_te.RemovePlugFromCar),
                state=dfc_te.RemovePlugFromCar(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.plug_in_car_post_connect: state_name(dfc_te.MovePlugToBattery),
                             out.Common.stop:                                     state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_te.MovePlugToBattery),
                state=dfc_te.MovePlugToBattery(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.plug_in_bat_post_obs: state_name(dfc_te.MovePlugToBatteryPreConnect),
                             out.Common.stop:                                 state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_te.MovePlugToBatteryPreConnect),
                state=dfc_te.MovePlugToBatteryPreConnect(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.plug_in_bat_pre_connect: state_name(dfc_te.InsertPlugToBattery),
                             out.Common.stop:                                    state_name(com.Stop)},
            )
            StateMachine.add(
                label=state_name(dfc_te.InsertPlugToBattery),
                state=dfc_te.InsertPlugToBattery(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.plug_in_bat_connect: state_name(dfc_te.ReleasePlugOnBattery),
                             out.Common.stop:                                state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_te.ReleasePlugOnBattery),
                state=dfc_te.ReleasePlugOnBattery(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.arm_in_bat_post_connect: state_name(dfc_te.MoveArmToDrivePos),
                             out.Common.stop:                                    state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(dfc_te.MoveArmToDrivePos),
                state=dfc_te.MoveArmToDrivePos(self.config, pilot),
                transitions={out.DisconnectFromCarTwist.arm_in_driving_pose: out.DisconnectFromCarTwist.arm_in_driving_pose}
            )
            StateMachine.add(
                label=state_name(com.Stop),
                state=com.Stop(self.config, pilot),
                transitions={out.Common.stop: out.Common.stop}
            )


class DisconnectFromCarElectric(DisconnectFromCar):

    def __init__(self, name: str, cfg_fp: Path, dtt_dir: Path) -> None:
        super().__init__(name, cfg_fp, dtt_dir)

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
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc_e.MoveArmToCarPreGrasp),
                state=dfc_e.MoveArmToCarPreGrasp(self.config, pilot),
                transitions={out.DisconnectFromCarElectric.arm_in_car_pre_connect: state_name(dfc_e.GraspPlugOnCar),
                             out.Common.stop:                                      state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
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
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(dfc_e.MovePlugToBatteryPreConnect),
                state=dfc_e.MovePlugToBatteryPreConnect(self.config, pilot),
                transitions={out.DisconnectFromCarElectric.plug_in_bat_pre_connect: state_name(dfc_e.InsertPlugToBattery),
                             out.Common.stop:                                       state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
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
