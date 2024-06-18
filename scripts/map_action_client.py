
from __future__ import annotations
from http import client

# libs
import rospy
import actionlib

# actions
from chargepal_actions.msg import (
    PlugInAdsAcAction,
    PlugInAdsDcAction,
    PlugInBcsAcAction,
    PlugInDskDmAction,
    PlugOutAdsAcAction,
    PlugOutAdsDcAction,
    PlugOutBcsAcAction,
    PlugOutDskDmAction,
    MoveHomeArmAction,
    FreeDriveArmAction,
    MarkerSocketCalibAdsAction,
    MarkerSocketCalibBcsAction,

    PlugInAdsAcGoal,
    PlugInAdsDcGoal,
    PlugInBcsAcGoal,
    PlugInDskDmGoal,
    PlugOutAdsAcGoal,
    PlugOutAdsDcGoal,
    PlugOutBcsAcGoal,
    PlugOutDskDmGoal,
    FreeDriveArmGoal,
    MoveHomeArmGoal,
    MarkerSocketCalibAdsGoal,
    MarkerSocketCalibBcsGoal,

    PlugInAdsAcResult,
    PlugInAdsDcResult,
    PlugInBcsAcResult,
    PlugInDskDmResult,
    PlugOutAdsAcResult,
    PlugOutAdsDcResult,
    PlugOutBcsAcResult,
    PlugOutDskDmResult,
    FreeDriveArmResult,
    MoveHomeArmResult,
    MarkerSocketCalibAdsResult,
    MarkerSocketCalibBcsResult,

    PlugInAdsAcFeedback,
    PlugInAdsDcFeedback,
    PlugInBcsAcFeedback,
    PlugInDskDmFeedback,
    PlugOutAdsAcFeedback,
    PlugOutAdsDcFeedback,
    PlugOutBcsAcFeedback,
    PlugOutDskDmFeedback,
    FreeDriveArmFeedback,
    MoveHomeArmFeedback,
    MarkerSocketCalibAdsFeedback,
    MarkerSocketCalibBcsFeedback,
)


# typing
from typing import Type
from genpy import Message


class ManipulationActionClient:

    def __init__(self, 
                 name: str,
                 act_msg: Type[Message],
                 goal_msg: Type[Message],
                 res_msg: Type[Message],
                 fb_msg: Type[Message],) -> None:
        self.goal_msg, self.res_msg, self.fb_msg = goal_msg, res_msg, fb_msg
        self.client = actionlib.SimpleActionClient(name, act_msg)
        self.client.wait_for_server()

    def run(self) -> Message:
        goal = self.goal_msg()
        self.client.send_goal(goal)
        self.client.wait_for_result()
        return client.get_result()
    


if __name__ == '__main__':
    
    plug_in_client_config = {
        'name': 'plug_in_dsk_dm', 
        'act_msg': PlugInDskDmAction, 
        'goal_msg': PlugInDskDmGoal,
        'res_msg': PlugInDskDmResult,
        'fb_msg': PlugInDskDmFeedback,
    }
    
    try:
        rospy.init_node('chargepal_map_action_client')
        plug_in_client = ManipulationActionClient(**plug_in_client_config)
        plug_in_res = plug_in_client.run()
        print(f"Plug-in process successfully: {plug_in_res.success}")
    except rospy.ROSInterruptException:
        print(f"Program interrupted before completion")
