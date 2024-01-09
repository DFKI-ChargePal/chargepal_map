
# libs
import rospy
import rospkg
from pathlib import Path
from chargepal_map import manipulation_action_processor


class ManipulationActionServer:

    _ur_pilot_cfg =  'green_type2.toml'

    def __init__(self) -> None:
        # Build configuration path
        ros_pack = rospkg.RosPack()
        ros_path = ros_pack.get_path('chargepal_map')
        self._cfk_dir = Path(ros_path).joinpath('config')
        self._proc_cfg_dir = self._cfk_dir.joinpath('process')
        # Create action processors
        manipulation_action_processor.create('connect_to_car', self._proc_cfg_dir)
        manipulation_action_processor.create('disconnect_from_car', self._proc_cfg_dir)


if __name__ == '__main__':
    rospy.init_node('manipulation_action_server')
    rospy.loginfo(f"Starting manipulation action servers")
    _ = ManipulationActionServer()
    rospy.loginfo(f"Ready to receive action goal")
    rospy.spin()
