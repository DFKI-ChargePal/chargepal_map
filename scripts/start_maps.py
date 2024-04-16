from __future__ import annotations

# libs
import sys
import yaml
import rospy
import logging
import ur_pilot
import camera_kit as ck
from pathlib import Path
import chargepal_map as mp

# typing
from typing import Any


def start_maps(fp_cfg: Path) -> None:
    """ Main function to start manipulation processes

    Args:
        fp_cfg: File path pointing to the configuration file

    Raises:
        RuntimeError: If configuration is not readable
    """
    # Create robot interface
    config_dir = fp_cfg.parent
    with fp_cfg.open('r') as fp:
        try:
            config_dict: dict[str, Any] = yaml.safe_load(fp)
        except Exception as e:
            raise RuntimeError(f"Error while reading {fp_cfg} configuration with error msg: {e}")

    # Setting up end-effector camera
    cam_dir = config_dir.joinpath('camera_info')
    cam_cc_path = cam_dir.joinpath(config_dict['camera']['cc'])
    cam = ck.camera_factory.create(config_dict['camera']['name'])
    cam.load_coefficients(cam_cc_path)
    # Setting up ur-pilot
    arm_dir = config_dir.joinpath('ur_arm')
    ur_pilot.logger.set_logging_level(logging.DEBUG)
    pilot = ur_pilot.Pilot(config_dir=arm_dir)
    pilot.register_ee_cam(cam, cam_dir)
    # Create manipulation state machine / process
    sm =  mp.ManipulationStateMachine(config_dir, config_dict)
    sm.build(pilot)
    # Create action servers
    for job_name in config_dict['jobs']:
        if mp.job_ids.is_valid(job_name):
            rospy.loginfo(f"Start action server: {job_name}")
            mp.manipulation_action_server.create(job_name, sm)
        else:
            raise ValueError(f"Invalid job with name: {job_name}.")


if __name__ == '__main__':
    rospy.init_node('manipulation_action_process', log_level=rospy.DEBUG)
    rospy.loginfo(f"Starting manipulation action process servers")
    sys_cfg_path = Path(sys.argv[1])
    if sys_cfg_path.exists() and sys_cfg_path.is_file():
        start_maps(sys_cfg_path)
    else:
        raise FileNotFoundError(f"Can not find configuration file '{sys_cfg_path}'")
    rospy.loginfo(f"Ready to receive action goal commands")
    rospy.spin()
