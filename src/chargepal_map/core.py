from __future__ import annotations

# libs
import rospy
import actionlib
from smach import UserData
from collections import namedtuple
from chargepal_map.state_machine.outcomes import out
from chargepal_map.state_machine.state_machine import ManipulationStateMachine

# actions
from chargepal_actions.msg import (
    PlugInAdsAcAction,
    PlugInAdsDcAction,
    PlugInBcsAcAction,
    PlugOutAdsAcAction,
    PlugOutAdsDcAction,
    PlugOutBcsAcAction,
    
    PlugInAdsAcGoal,
    PlugInAdsDcGoal,
    PlugInBcsAcGoal,
    PlugOutAdsAcGoal,
    PlugOutAdsDcGoal,
    PlugOutBcsAcGoal,

    PlugInAdsAcResult,
    PlugInAdsDcResult,
    PlugInBcsAcResult,
    PlugOutAdsAcResult,
    PlugOutAdsDcResult,
    PlugOutBcsAcResult,

    PlugInAdsAcFeedback,
    PlugInAdsDcFeedback,
    PlugInBcsAcFeedback,
    PlugOutAdsAcFeedback,
    PlugOutAdsDcFeedback,
    PlugOutBcsAcFeedback,
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
        """Base class of a Manipulation Action Server 
        """
        self.name = name
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
        rospy.loginfo(f"Launch the plug-in process to the adapter station with AC plug")
        result_msg = self.res_msg()
        try:
            ud = UserData()
            ud.job_id = self.name
            outcome = self.sm.execute(ud)
            if outcome == out.completed:
                result_msg.success = True
                self.act_srv.set_succeeded(result=result_msg)
                rospy.loginfo(f"Finish plug-in process successfully.")
            else:
                result_msg.success = False
                self.act_srv.set_preempted(result=result_msg)
                rospy.loginfo(f"Stop plug-in process prematurely.")
        except Exception as e:
            rospy.logwarn(f"Error while plug-in process: {e}")
            result_msg.success = False
            self.act_srv.set_aborted(result=result_msg)

    def wait_for_usr_feedback(self) -> None:
        feedback = self.fb_msg()
        feedback.status = "wait_for_user"
        self.action_server.publish_feedback(feedback)


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


class JobNames:

    plug_in_ads_ac = 'plug_in_ads_ac'
    plug_in_ads_dc = 'plug_in_ads_dc'
    plug_in_bcs_ac = 'plug_in_bcs_ac'

    plug_out_ads_ac = 'plug_out_ads_ac'
    plug_out_ads_dc = 'plug_out_ads_dc'
    plug_out_bcs_ac = 'plug_out_bcs_ac'

    def plug_in() -> list[str]:
        """ Get all jobs which refer to the plug-in process

        Returns:
            List with valid job names
        """
        return [JobNames.plug_in_ads_ac, JobNames.plug_in_ads_dc, JobNames.plug_in_bcs_ac]
    
    def plug_out() -> list[str]:
        """ Get all jobs which refer to the plug-out process

        Returns:
            List with valid job names
        """
        return [JobNames.plug_out_ads_ac, JobNames.plug_out_ads_dc, JobNames.plug_out_bcs_ac]

    def workspace_left() -> list[str]:
        """ Get all jobs which have to be executed in the left workspace

        Returns:
            List with valid job names
        """
        return [JobNames.plug_in_ads_ac, JobNames.plug_in_bcs_ac, JobNames.plug_out_ads_ac, JobNames.plug_out_bcs_ac]
    
    def workspace_right() -> list[str]:
        """ Get all jobs which have to be executed in the right workspace

        Returns:
            List with valid job names
        """
        return [JobNames.plug_in_ads_dc, JobNames.plug_out_ads_dc]

    def type2_female() -> list[str]:
        """ Get all jobs which uses the 'Type2' (AC) plug with female inlet

        Returns:
            List with valid job names
        """
        return [JobNames.plug_in_ads_ac, JobNames.plug_out_ads_ac]
    
    def type2_male() -> list[str]:
        """ Get all jobs which uses the 'Type2' (AC) plug with male inlet

        Returns:
            List with valid job names
        """
        return [JobNames.plug_in_bcs_ac, JobNames.plug_out_bcs_ac]
    
    def ccs_female() -> list[str]:
        """ Get all jobs which uses the 'CCS' (DC) plug with female inlet

        Returns:
            List with valid job names
        """
        return [JobNames.plug_in_ads_dc, JobNames.plug_out_ads_dc]


# Register all manipulation action servers
job_ids = JobNames()
manipulation_action_server = ActSrvFactory()

manipulation_action_server.register(job_ids.plug_in_ads_ac,
                                    PlugInAdsAcAction,
                                    PlugInAdsAcGoal,
                                    PlugInAdsAcResult,
                                    PlugInAdsAcFeedback)

manipulation_action_server.register(job_ids.plug_in_ads_dc,
                                    PlugInAdsDcAction,
                                    PlugInAdsDcGoal,
                                    PlugInAdsDcResult,
                                    PlugInAdsDcFeedback)

manipulation_action_server.register(job_ids.plug_in_bcs_ac,
                                    PlugInBcsAcAction,
                                    PlugInBcsAcGoal,
                                    PlugInBcsAcResult,
                                    PlugInBcsAcFeedback)

manipulation_action_server.register(job_ids.plug_out_ads_ac,
                                    PlugOutAdsAcAction,
                                    PlugOutAdsAcGoal,
                                    PlugOutAdsAcResult,
                                    PlugOutAdsAcFeedback)

manipulation_action_server.register(job_ids.plug_out_ads_dc,
                                    PlugOutAdsDcAction,
                                    PlugOutAdsDcGoal,
                                    PlugOutAdsDcResult,
                                    PlugOutAdsDcFeedback)

manipulation_action_server.register(job_ids.plug_out_bcs_ac,
                                    PlugOutBcsAcAction,
                                    PlugOutBcsAcGoal,
                                    PlugOutBcsAcResult,
                                    PlugOutBcsAcFeedback)
