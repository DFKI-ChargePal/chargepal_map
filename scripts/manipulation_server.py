
# libs
import rospy
import rospkg
import actionlib
from pathlib import Path
from chargepal_map import manipulation_action_processor

# actions
from chargepal_actions.msg import (
    ConnectPlugToCarAction, 
    ConnectPlugToCarActionGoal,
    DisconnectPlugFromCarAction,
    DisconnectPlugFromCarActionGoal,
)



class ManipulationActionServer:

    _ur_pilot_cfg =  'green_type2.toml'

    def __init__(self) -> None:
        # Build configuration path
        ros_pack = rospkg.RosPack()
        ros_path = ros_pack.get_path('chargepal_map')
        self._cfk_dir = Path(ros_path).joinpath('config')
        # Initialize action server
        self._ctc_as = actionlib.SimpleActionServer(
            'connect_to_car', ConnectPlugToCarAction, self.connect_to_car, False)
        self._dfc_as = actionlib.SimpleActionServer(
            'disconnect_from_car', DisconnectPlugFromCarAction, self.disconnect_from_car, False)
        self._ctc_as.start()
        self._dfc_as.start()
        # Create state machine
        self._proc_cfg_dir = self._cfk_dir.joinpath('process')
        self.ctc_process = manipulation_action_processor.create('connect_to_car', self._proc_cfg_dir)
        self.dfc_process = manipulation_action_processor.create('disconnect_from_car', self._proc_cfg_dir)

    def connect_to_car(self, goal: ConnectPlugToCarActionGoal) -> None:
        rospy.loginfo(f"Approach connect plug to car process")
        try:
            rospy.loginfo(f"Process connect task step by step")
            # Execute SMACH plan
            outcome = self.ctc_process.execute()
            self._ctc_as.set_succeeded()
            rospy.loginfo(f"Finish connect process successfully.")
        except Exception as e:
            rospy.logwarn(f"Error while plugging process: {e}")
            self._ctc_as.set_aborted()
        rospy.loginfo(f"Leaving connect plug to car process")

    def disconnect_from_car(self, goal: DisconnectPlugFromCarActionGoal) -> None:
        rospy.loginfo(f"Approach disconnect plug from car process")
        try:
            rospy.loginfo(f"Process disconnect task step by step")
            # Execute SMACH plan
            outcome = self.dfc_process.execute()
            self._dfc_as.set_succeeded()
            rospy.loginfo(f"Finish disconnect process successfully.")
        except Exception as e:
            rospy.logwarn(f"Error while plugging process: {e}")
            self._dfc_as.set_aborted()
        rospy.loginfo(f"Leaving disconnect plug from car process")


if __name__ == '__main__':
    rospy.init_node('manipulation_action_server')
    rospy.loginfo(f"Starting manipulation action servers")
    _ = ManipulationActionServer()
    rospy.loginfo(f"Ready to receive action goal")
    rospy.spin()
