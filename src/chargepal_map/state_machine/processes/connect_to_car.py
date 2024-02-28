from __future__ import annotations

# global
import rospy
import actionlib
from pathlib import Path
from smach import StateMachine

# local
import chargepal_map.state_machine.outcomes as out
import chargepal_map.state_machine.states.common as com
from chargepal_map.state_machine.processes.process import ProcessABC
import chargepal_map.state_machine.states.connect_to_car_twist as ctc_t
import chargepal_map.state_machine.states.connect_to_car_electric as ctc_e
from chargepal_map.state_machine.utils import state_name

# actions
from chargepal_actions.msg import (
    ConnectPlugToCarAction, 
    ConnectPlugToCarActionGoal,
    ConnectPlugToCarActionFeedback
)

# typing
from ur_pilot import Pilot


class ConnectToCar(ProcessABC):

    def __init__(self, name: str, cfg_fp: Path, dtt_dir: Path) -> None:
        super().__init__('connect_to_car', cfg_fp, dtt_dir)
        self.state_machine = StateMachine(outcomes=[out.Common.stop, out.ConnectToCarTwist.arm_in_driving_pose])
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
        raise NotImplementedError('Must be implemented in subclass')


class ConnectToCarTwist(ConnectToCar):

    def __init__(self, name: str, cfg_fp: Path, dtt_dir: Path) -> None:
        super().__init__(name, cfg_fp, dtt_dir)

    def set_up(self, pilot: Pilot) -> None:
        # Open smash container to add states and transitions
        with self.state_machine:
            StateMachine.add(
                label=state_name(ctc_t.MoveArmToBattery),
                state=ctc_t.MoveArmToBattery(self.config, pilot),
                transitions={out.ConnectToCarTwist.arm_in_bat_pre_obs: state_name(ctc_t.ObservePlugOnBattery),
                             out.Common.stop:                          state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_t.ObservePlugOnBattery),
                state=ctc_t.ObservePlugOnBattery(self.config, pilot),
                transitions={out.ConnectToCarTwist.arm_in_bat_post_obs: state_name(ctc_t.MoveArmToBatteryPreGrasp),
                             out.Common.stop:                           state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc_t.MoveArmToBatteryPreGrasp),
                state=ctc_t.MoveArmToBatteryPreGrasp(self.config, pilot),
                transitions={out.ConnectToCarTwist.arm_in_bat_pre_connect: state_name(ctc_t.GraspPlugOnBattery),
                             out.Common.stop:                              state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc_t.GraspPlugOnBattery),
                state=ctc_t.GraspPlugOnBattery(self.config, pilot),
                transitions={out.ConnectToCarTwist.plug_in_bat_connect: state_name(ctc_t.RemovePlugFromBattery),
                             out.Common.stop:                           state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_t.RemovePlugFromBattery),
                state=ctc_t.RemovePlugFromBattery(self.config, pilot),
                transitions={out.ConnectToCarTwist.plug_in_bat_post_connect: state_name(ctc_t.MovePlugToCar),
                             out.Common.stop:                                state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_t.MovePlugToCar),
                state=ctc_t.MovePlugToCar(self.config, pilot),
                transitions={out.ConnectToCarTwist.plug_in_car_pre_obs: state_name(ctc_t.ObserveSocketOnCar),
                             out.Common.stop:                           state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_t.ObserveSocketOnCar),
                state=ctc_t.ObserveSocketOnCar(self.config, pilot),
                transitions={out.ConnectToCarTwist.plug_in_car_post_obs: state_name(ctc_t.MovePlugToCarPreConnect),
                             out.Common.stop:                            state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc_t.MovePlugToCarPreConnect),
                state=ctc_t.MovePlugToCarPreConnect(self.config, pilot),
                transitions={out.ConnectToCarTwist.plug_in_car_pre_connect: state_name(ctc_t.InsertPlugToCar),
                             out.Common.stop:                               state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc_t.InsertPlugToCar),
                state=ctc_t.InsertPlugToCar(self.config, pilot),
                transitions={out.ConnectToCarTwist.plug_in_car_connect: state_name(ctc_t.ReleasePlugOnCar),
                             out.Common.stop:                           state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_t.ReleasePlugOnCar),
                state=ctc_t.ReleasePlugOnCar(self.config, pilot),
                transitions={out.ConnectToCarTwist.arm_in_car_post_connect: state_name(ctc_t.MoveArmToDrivePos),
                             out.Common.stop:                               state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_t.MoveArmToDrivePos),
                state=ctc_t.MoveArmToDrivePos(self.config, pilot),
                transitions={out.ConnectToCarTwist.arm_in_driving_pose: out.ConnectToCarTwist.arm_in_driving_pose}
            )
            StateMachine.add(
                label=state_name(com.Stop),
                state=com.Stop(self.config, pilot),
                transitions={out.Common.stop: out.Common.stop}
            )


class ConnectToCarTwistEasy(ConnectToCar):

    def __init__(self, name: str, cfg_fp: Path, dtt_dir: Path) -> None:
        super().__init__(name, cfg_fp, dtt_dir)

    def set_up(self, pilot: Pilot) -> None:
        # Open smash container to add states and transitions
        with self.state_machine:
            StateMachine.add(
                label=state_name(ctc_t.MoveArmToBattery),
                state=ctc_t.MoveArmToBattery(self.config, pilot),
                transitions={out.ConnectToCarTwist.arm_in_bat_pre_obs: state_name(ctc_t.MoveArmToBatteryPreGrasp),
                             out.Common.stop:                          state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_t.MoveArmToBatteryPreGrasp),
                state=ctc_t.MoveArmToBatteryPreGrasp(self.config, pilot),
                transitions={out.ConnectToCarTwist.arm_in_bat_pre_connect: state_name(ctc_t.GraspPlugOnBattery),
                             out.Common.stop:                              state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_t.GraspPlugOnBattery),
                state=ctc_t.GraspPlugOnBattery(self.config, pilot),
                transitions={out.ConnectToCarTwist.plug_in_bat_connect: state_name(ctc_t.RemovePlugFromBattery),
                             out.Common.stop:                           state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_t.RemovePlugFromBattery),
                state=ctc_t.RemovePlugFromBattery(self.config, pilot),
                transitions={out.ConnectToCarTwist.plug_in_bat_post_connect: state_name(ctc_t.MovePlugToCar),
                             out.Common.stop:                                state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_t.MovePlugToCar),
                state=ctc_t.MovePlugToCar(self.config, pilot),
                transitions={out.ConnectToCarTwist.plug_in_car_pre_obs: state_name(ctc_t.ObserveSocketOnCar),
                             out.Common.stop:                           state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_t.ObserveSocketOnCar),
                state=ctc_t.ObserveSocketOnCar(self.config, pilot),
                transitions={out.ConnectToCarTwist.plug_in_car_post_obs: state_name(ctc_t.MovePlugToCarPreConnect),
                             out.Common.stop:                            state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc_t.MovePlugToCarPreConnect),
                state=ctc_t.MovePlugToCarPreConnect(self.config, pilot),
                transitions={out.ConnectToCarTwist.plug_in_car_pre_connect: state_name(ctc_t.InsertPlugToCar),
                             out.Common.stop:                               state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc_t.InsertPlugToCar),
                state=ctc_t.InsertPlugToCar(self.config, pilot),
                transitions={out.ConnectToCarTwist.plug_in_car_connect: state_name(ctc_t.ReleasePlugOnCar),
                             out.Common.stop:                           state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_t.ReleasePlugOnCar),
                state=ctc_t.ReleasePlugOnCar(self.config, pilot),
                transitions={out.ConnectToCarTwist.arm_in_car_post_connect: state_name(ctc_t.MoveArmToDrivePos),
                             out.Common.stop:                               state_name(com.Stop)}
            )
            StateMachine.add(
                label=state_name(ctc_t.MoveArmToDrivePos),
                state=ctc_t.MoveArmToDrivePos(self.config, pilot),
                transitions={out.ConnectToCarTwist.arm_in_driving_pose: out.ConnectToCarTwist.arm_in_driving_pose}
            )
            StateMachine.add(
                label=state_name(com.Stop),
                state=com.Stop(self.config, pilot),
                transitions={out.Common.stop: out.Common.stop}
            )


class ConnectToCarElectric(ConnectToCar):

    def __init__(self, name: str, cfg_fp: Path, dtt_dir: Path) -> None:
        super().__init__(name, cfg_fp, dtt_dir)

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
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc_e.MoveArmToBatteryPreGrasp),
                state=ctc_e.MoveArmToBatteryPreGrasp(self.config, pilot),
                transitions={out.ConnectToCarElectric.arm_in_bat_pre_connect: state_name(ctc_e.GraspPlugOnBattery),
                             out.Common.stop:                                 state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
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
                remapping={'T_base2socket': 'T_base2socket'}
            )
            StateMachine.add(
                label=state_name(ctc_e.MovePlugToCarPreConnect),
                state=ctc_e.MovePlugToCarPreConnect(self.config, pilot),
                transitions={out.ConnectToCarElectric.plug_in_car_pre_connect: state_name(ctc_e.InsertPlugToCar),
                             out.Common.stop:                                  state_name(com.Stop)},
                remapping={'T_base2socket': 'T_base2socket'}
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
