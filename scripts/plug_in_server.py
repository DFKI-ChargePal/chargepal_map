
# libs
import rospy
import rospkg
import ur_pilot
import actionlib
import chargepal_map
from pathlib import Path

# actions
from chargepal_actions.msg import PlugInAction, PlugInActionGoal
from chargepal_actions.msg import PlugInRecoveryAction, PlugInRecoveryActionGoal



class PlugInActionServer:

    _ur_pilot_cfg =  'green_type2.toml'

    def __init__(self) -> None:
        # Build configuration path
        ros_pack = rospkg.RosPack()
        ros_path = ros_pack.get_path('chargepal_map')
        self._cfk_dir = Path(ros_path).joinpath('config')
        
        # Initialize action server
        self._plug_in_as = actionlib.SimpleActionServer('plug_in', PlugInAction, self.forward, False)
        self._recover_as = actionlib.SimpleActionServer('plug_in_recovery', PlugInRecoveryAction, self.recover, False)
        self._plug_in_as.start()
        self._recover_as.start()
        # Create robot interface
        self._ur_pilot = ur_pilot.Pilot(self._cfk_dir.joinpath(self._ur_pilot_cfg))
        # Create state machine
        self.ctc_process = chargepal_map.ProcessFactory.create('connect_to_car')


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
