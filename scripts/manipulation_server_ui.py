from __future__ import annotations

import rospy
# from chargepal_map import ui


if __name__ == '__main__':
    rospy.init_node('state_machine_user_interface')
    rospy.loginfo(f"Starting user interface to control the state machine process.")
    # usr_interface = ui.create()
    rospy.loginfo(f"Ready to submit user requests.")
    # usr_interface.run()
