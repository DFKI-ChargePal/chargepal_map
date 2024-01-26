from __future__ import annotations

# libs
import sys
import yaml
import rospy
import ur_pilot
import camera_kit as ck
from pathlib import Path

from chargepal_map import manipulation_action_processor

# typing
from typing import Any



class ManipulationActionServer:

    def __init__(self, fp_config: Path) -> None:
        # Create robot interface
        dir_config = fp_config.parent
        with fp_config.open('r') as fp:
            try:
                config_raw: dict[str, Any] = yaml.safe_load(fp)
            except Exception as e:
                raise RuntimeError(f"Error while reading {fp_config} configuration with error msg: {e}")

        # Setting up end-effector camera
        cam_dir = dir_config.joinpath('camera_info')
        cam_cc_path = cam_dir.joinpath(config_raw['camera']['cc'])
        cam = ck.camera_factory.create(config_raw['camera']['name'])
        cam.load_coefficients(cam_cc_path)
        # Setting up ur-pilot
        pilot_cfg_path = dir_config.joinpath('ur_arm', config_raw['ur_arm']['config_file'])
        self.ur_pilot = ur_pilot.Pilot(pilot_cfg_path)
        self.ur_pilot.robot.register_ee_cam(cam, cam_dir)
        # Create detector directory path
        dtt_path = dir_config.joinpath('cv_detector')
        # Create action processors
        proc_cfg_dir = dir_config.joinpath('process')
        for proc_name, proc_fp in config_raw['process'].items():
            manipulation_action_processor.create(proc_name, proc_cfg_dir.joinpath(proc_fp), self.ur_pilot, dtt_path)


if __name__ == '__main__':
    rospy.init_node('manipulation_action_server')
    rospy.loginfo(f"Starting manipulation action servers")
    sys_cfg_path = Path(sys.argv[1])
    if sys_cfg_path.exists() and sys_cfg_path.is_file():
        ManipulationActionServer(sys_cfg_path)
    else:
        raise FileNotFoundError(f"Can not find configuration file '{sys_cfg_path}'")
    rospy.loginfo(f"Ready to receive action goal")
    rospy.spin()
