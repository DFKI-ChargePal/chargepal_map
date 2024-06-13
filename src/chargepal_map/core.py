from __future__ import annotations

# libs
import rospy
import actionlib
from smach import UserData
from collections import namedtuple

from chargepal_map.job import Job
from chargepal_map.state_machine import outcomes as out
from chargepal_map.state_machine.utils import StateMachineError
from chargepal_map.state_machine.state_machine import ManipulationStateMachine

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


class ManipulationActionServer:

    def __init__(self, 
                 name: str,
                 act_msg: Type[Message],
                 goal_msg: Type[Message],
                 res_msg: Type[Message],
                 fb_msg: Type[Message],
                 state_machine: ManipulationStateMachine) -> None:
        """ Base class of a Manipulation Action Server """
        self.name = name
        self.shutdown = False  # Flag to signal shutdown
        self.sm = state_machine
        self.act_msg = act_msg
        self.goal_msg, self.res_msg, self.fb_msg = goal_msg, res_msg, fb_msg
        self.act_srv = actionlib.SimpleActionServer(name=self.name,
                                                    ActionSpec=self.act_msg,
                                                    execute_cb=self.action_callback,
                                                    auto_start=False)
        self.act_srv.start()

    def action_callback(self, goal: Message) -> None:
        """ Callback function for the ROS ActionServer class. Is called when publishing to the action goal topic

        Args:
            goal: Specific action goal message
        """
        rospy.loginfo(f"Launch a new process.")
        result_msg = self.res_msg()
        try:
            ud = UserData()
            ud.job = Job(self.name)
            outcome = self.sm.execute(ud)
            if outcome == out.job_complete:
                result_msg.success = True
                self.act_srv.set_succeeded(result=result_msg)
                rospy.loginfo(f"Finish process successfully with outcome {outcome}.")
            else:
                result_msg.success = False
                self.act_srv.set_preempted(result=result_msg)
                rospy.loginfo(f"Stop process prematurely with outcome {outcome}.")
        except StateMachineError as sme:
            self.act_srv.set_aborted(result=result_msg)
            result_msg.success = False
            rospy.logwarn(f"Error in state machine procedure: {sme}")
        except Exception as e:
            self.act_srv.set_aborted(result=result_msg)
            result_msg.success = False
            self.shutdown = True
            rospy.logerr(f"Unknown error while executing manipulation process: {e}")
            rospy.logerr(f"Shutdown manipulation node!")

    def wait_for_usr_feedback(self) -> None:
        feedback = self.fb_msg()
        feedback.status = "wait_for_user"
        self.act_srv.publish_feedback(feedback)


class ActSrvFactory:

    MsgCollection = namedtuple("MsgCollection", "act_msg goal_msg res_msg fb_msg")

    def __init__(self) -> None:
        self._selection: dict[str, ActSrvFactory.MsgCollection] = {}

    def register(self,
                 name: str,
                 act_msg: Type[Message],
                 goal_msg: Type[Message],
                 res_msg: Type[Message],
                 fb_msg: Type[Message]) -> None:
        """ Class method to register a new action server with its specific message types

        Args:
            name:     Name of the action server. 
                      Has to match with the configuration file name in is also used as job key internally.
            act_msg:  Type of the action message
            goal_msg: Type of the action goal message
            res_msg:  Type of the action result message
            fb_msg:   Type of the action feedback message
        """
        msg_col = ActSrvFactory.MsgCollection(act_msg=act_msg, goal_msg=goal_msg, res_msg=res_msg, fb_msg=fb_msg)
        self._selection[name] = msg_col

    def create(self, name: str, state_machine: ManipulationStateMachine) -> ManipulationActionServer:
        msg_col = self._selection.get(name)
        if msg_col is None:
            raise KeyError(f"Unknown or unregistered ManipulationActionServer '{name}'"
                           f"Available server are: {list(self._selection.keys())}")
        act_srv = ManipulationActionServer(name,
                                           msg_col.act_msg,
                                           msg_col.goal_msg,
                                           msg_col.res_msg, 
                                           msg_col.fb_msg,
                                           state_machine)
        return act_srv





# Register all manipulation action servers
manipulation_action_server = ActSrvFactory()

# PLUG IN
manipulation_action_server.register(Job.ID.plug_in_ads_ac,
                                    PlugInAdsAcAction,
                                    PlugInAdsAcGoal,
                                    PlugInAdsAcResult,
                                    PlugInAdsAcFeedback)

manipulation_action_server.register(Job.ID.plug_in_ads_dc,
                                    PlugInAdsDcAction,
                                    PlugInAdsDcGoal,
                                    PlugInAdsDcResult,
                                    PlugInAdsDcFeedback)

manipulation_action_server.register(Job.ID.plug_in_bcs_ac,
                                    PlugInBcsAcAction,
                                    PlugInBcsAcGoal,
                                    PlugInBcsAcResult,
                                    PlugInBcsAcFeedback)

manipulation_action_server.register(Job.ID.plug_in_dsk_dm,
                                    PlugInDskDmAction,
                                    PlugInDskDmGoal,
                                    PlugInDskDmResult,
                                    PlugInDskDmFeedback)

# PLUG OUT
manipulation_action_server.register(Job.ID.plug_out_ads_ac,
                                    PlugOutAdsAcAction,
                                    PlugOutAdsAcGoal,
                                    PlugOutAdsAcResult,
                                    PlugOutAdsAcFeedback)

manipulation_action_server.register(Job.ID.plug_out_ads_dc,
                                    PlugOutAdsDcAction,
                                    PlugOutAdsDcGoal,
                                    PlugOutAdsDcResult,
                                    PlugOutAdsDcFeedback)

manipulation_action_server.register(Job.ID.plug_out_bcs_ac,
                                    PlugOutBcsAcAction,
                                    PlugOutBcsAcGoal,
                                    PlugOutBcsAcResult,
                                    PlugOutBcsAcFeedback)

manipulation_action_server.register(Job.ID.plug_out_dsk_dm,
                                    PlugOutDskDmAction,
                                    PlugOutDskDmGoal,
                                    PlugOutDskDmResult,
                                    PlugOutDskDmFeedback)

# MISCELLANEOUS
manipulation_action_server.register(Job.ID.free_drive_arm,
                                    FreeDriveArmAction,
                                    FreeDriveArmGoal,
                                    FreeDriveArmResult, 
                                    FreeDriveArmFeedback)

manipulation_action_server.register(Job.ID.move_home_arm, 
                                    MoveHomeArmAction, 
                                    MoveHomeArmGoal,
                                    MoveHomeArmResult,
                                    MoveHomeArmFeedback)

manipulation_action_server.register(Job.ID.marker_socket_calib_ads,
                                    MarkerSocketCalibAdsAction,
                                    MarkerSocketCalibAdsGoal,
                                    MarkerSocketCalibAdsResult,
                                    MarkerSocketCalibAdsFeedback)

manipulation_action_server.register(Job.ID.marker_socket_calib_bcs,
                                    MarkerSocketCalibBcsAction,
                                    MarkerSocketCalibBcsGoal,
                                    MarkerSocketCalibBcsResult,
                                    MarkerSocketCalibBcsFeedback)
