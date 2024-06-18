
from __future__ import annotations
from http import client

# libs
import rospy
import argparse
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


_plug_in_action_config = {
        'name': 'plug_in_dsk_dm', 
        'act_msg': PlugInDskDmAction, 
        'goal_msg': PlugInDskDmGoal,
        'res_msg': PlugInDskDmResult,
        'fb_msg': PlugInDskDmFeedback,
    }

_plug_out_action_config = {
        'name': 'plug_out_dsk_dm', 
        'act_msg': PlugOutDskDmAction, 
        'goal_msg': PlugOutDskDmGoal,
        'res_msg': PlugOutDskDmResult,
        'fb_msg': PlugOutDskDmFeedback,
    }



class ManipulationAction:

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
        return self.client.get_result()



if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Script to run MAP actions')
    parser.add_argument('loops', type=int, help='Number of loops that are performed')
    args = parser.parse_args()
    try:
        rospy.init_node('chargepal_map_action_client')
        plug_in_action = ManipulationAction(**_plug_in_action_config)
        plug_out_action = ManipulationAction(**_plug_out_action_config)

        n_loops: int = args.loops
        for l in range(args.loops):
            print(f"\n")
            print(f"----   Run Manipulation Process   ----")
            print(f"Start loop: {l+1}/{n_loops}")
            plug_in_res = plug_in_action.run()
            print(f"Plug-in process successfully: {plug_in_res.success}")
            stop_process = True
            if plug_in_res.success:
                plug_out_res = plug_out_action.run()
                print(f"Plug-out process successfully: {plug_out_res.success}")
                if not plug_out_res.success:
                    print(f"Error in plug-out process. Stop executing")
                else:
                    stop_process = False
            else:
                print(f"Error during plug-in process. Stop executing")
            print(f"--------------------------------------")
            if stop_process:
                break
        print(f"\n")
    except rospy.ROSInterruptException:
        print(f"Program interrupted before completion")
