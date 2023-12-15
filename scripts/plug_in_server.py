import rospy
import rospkg
import smach
import ur_pilot
import actionlib
from pathlib import Path
from smach import StateMachine

from chargepal_actions.msg import PlugInAction, PlugInActionGoal
from chargepal_actions.msg import PlugInRecoveryAction, PlugInRecoveryActionGoal

from chargepal_map import outcomes as out
from chargepal_map import connect_to_car as ctc
from chargepal_map.processes.utils import state_name

from chargepal_map.smach import Foo2FooAAbc, Bar


class PlugInActionServer:

    _cfg_name =  'default.toml'

    def __init__(self) -> None:
        # Build configuration path
        ros_pack = rospkg.RosPack()
        ros_path = ros_pack.get_path('chargepal_map')
        self._cfg_path = Path(ros_path).joinpath('config').joinpath(self._cfg_name)
        # Initialize action server
        self._plug_in_as = actionlib.SimpleActionServer('plug_in', PlugInAction, self.forward, False)
        self._recover_as = actionlib.SimpleActionServer('plug_in_recovery', PlugInRecoveryAction, self.recover, False)
        self._plug_in_as.start()
        self._recover_as.start()

        # Create robot interface
        self._ur_pilot = ur_pilot.Pilot(self._cfg_path)

        self.ctc_process = StateMachine(outcomes=[out.ConnectToCar.arm_in_driving_pose])
        # Open smash container to add states and transitions
        with self.ctc_process:
            StateMachine.add(
                label=state_name(ctc.MoveArmToBattery), 
                state=ctc.MoveArmToBattery(self._ur_pilot), 
                transitions={out.ConnectToCar.arm_in_bat_obs:state_name(ctc.ObservePlugOnBattery)}
                )
            StateMachine.add(
                label=state_name(ctc.ObservePlugOnBattery), 
                state=ctc.ObservePlugOnBattery(self._ur_pilot), 
                transitions={out.ConnectToCar.arm_in_bat_pre_connect:state_name(ctc.GraspPlugOnBattery)}
                )
            StateMachine.add(
                label=state_name(ctc.GraspPlugOnBattery), 
                state=ctc.GraspPlugOnBattery(self._ur_pilot), 
                transitions={out.ConnectToCar.plug_in_bat_connect:state_name(ctc.RemovePlugFromBattery)}
                )
            StateMachine.add(
                label=state_name(ctc.RemovePlugFromBattery), 
                state=ctc.RemovePlugFromBattery(self._ur_pilot), 
                transitions={out.ConnectToCar.plug_in_bat_post_connect:state_name(ctc.MovePlugToCar)}
                )
            StateMachine.add(
                label=state_name(ctc.MovePlugToCar), 
                state=ctc.MovePlugToCar(self._ur_pilot), 
                transitions={out.ConnectToCar.plug_in_car_obs:state_name(ctc.ObserveSocketOnCar)}
                )
            StateMachine.add(
                label=state_name(ctc.ObserveSocketOnCar), 
                state=ctc.ObserveSocketOnCar(self._ur_pilot), 
                transitions={out.ConnectToCar.plug_in_car_pre_connect:state_name(ctc.InsertPlugToCar)}
                )
            StateMachine.add(
                label=state_name(ctc.InsertPlugToCar), 
                state=ctc.InsertPlugToCar(self._ur_pilot), 
                transitions={out.ConnectToCar.plug_in_car_connect:state_name(ctc.ReleasePlugOnCar)}
                )
            StateMachine.add(
                label=state_name(ctc.ReleasePlugOnCar), 
                state=ctc.ReleasePlugOnCar(self._ur_pilot), 
                transitions={out.ConnectToCar.arm_in_car_post_connect:state_name(ctc.MoveArmToDrivePos)}
                )
            StateMachine.add(
                label=state_name(ctc.MoveArmToDrivePos), 
                state=ctc.MoveArmToDrivePos(self._ur_pilot), 
                # No transition. Stop when reaching this state
                )

    def forward(self, goal: PlugInActionGoal) -> None:
        rospy.loginfo(f"Approach plug-in action")
        try:
            rospy.loginfo(f"Process plug-in task step by step")
            # Execute SMACH plan
            outcome = self.ctc_process.execute()
            self._plug_in_as.set_succeeded()
            rospy.loginfo(f"Finish plug-in process successfully.")
        except Exception as e:
            rospy.logwarn(f"Error while plugging process: {e}")
            self._plug_in_as.set_aborted()
        rospy.loginfo(f"Leaving plug-in action")

    def recover(self, goal: PlugInRecoveryActionGoal) -> None:
        rospy.loginfo(f"Starting recovering behavior")

        rospy.loginfo(f"Finish recovering")
        self._recover_as.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('plug_in_server')
    rospy.loginfo(f"Starting plug-in action servers")
    _ = PlugInActionServer()
    rospy.loginfo(f"Ready to receive action goal")
    rospy.spin()
