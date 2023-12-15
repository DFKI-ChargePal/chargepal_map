import rospy
import rospkg
import smach
# import ur_pilot
import actionlib
from pathlib import Path

from chargepal_actions.msg import PlugInAction, PlugInActionGoal
from chargepal_actions.msg import PlugInRecoveryAction, PlugInRecoveryActionGoal

from chargepal_map.smach import Foo, Bar


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

        self.sm = smach.StateMachine(outcomes=['outcome4'])

        # self._ur_pilot = ur_pilot.Pilot(self._cfg_path)
        # self.state = ArmInDrivePose()

    def forward(self, goal: PlugInActionGoal) -> None:
        rospy.loginfo(f"Starting plug-in task")
        try:
            rospy.loginfo(f"Process plug-in task step by step")
            # Open the container
            with self.sm:
                # Add states to the container
                smach.StateMachine.add('FOO', Foo(), transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
                smach.StateMachine.add('BAR', Bar(), transitions={'outcome1':'FOO'})
            
            # Execute SMACH plan
            outcome = self.sm.execute()
            self._plug_in_as.set_succeeded()
        except Exception as e:
            rospy.logwarn(f"Error while plugging: {e}")
            self._plug_in_as.set_aborted()
        rospy.loginfo(f"Finish plug-in process")

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
