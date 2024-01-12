
# libs
import sys
import rospy
import rospkg
import ur_pilot
from pathlib import Path

from chargepal_map import manipulation_action_processor


class ManipulationActionServer:

    def __init__(self, cfg_path: Path) -> None:
        # Create robot interface
        arm_cfg_file = cfg_path.joinpath('ur_arm').joinpath('robot.toml')
        # self.ur_pilot = ur_pilot.Pilot(arm_cfg_file)
        # Create action processors
        proc_cfg_dir = cfg_path.joinpath('process')
        manipulation_action_processor.create('connect_to_car', proc_cfg_dir)
        manipulation_action_processor.create('disconnect_from_car', proc_cfg_dir)


if __name__ == '__main__':
    rospy.init_node('manipulation_action_server')
    rospy.loginfo(f"Starting manipulation action servers")
    sys_cfg_path = Path(sys.argv[1])
    if sys_cfg_path.exists():
        ManipulationActionServer(sys_cfg_path)
    else:
        raise NotADirectoryError(f"Can not find parent configuration folder under '{sys_cfg_path}'")
    rospy.loginfo(f"Ready to receive action goal")
    rospy.spin()
