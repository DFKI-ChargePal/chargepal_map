from __future__ import annotations

# libs
import abc
import rospy
import actionlib
from smach import UserData
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
from typing import Any, Type



class ManipulationActionServer(metaclass=abc.ABCMeta):

    def __init__(self, name: str, state_machine: ManipulationStateMachine) -> None:
        """Base class of a Manipulation Action Server 
        """
        self.name = name
        self.sm = state_machine

    @abc.abstractmethod
    def action_callback(self, goal: Any) -> None:
        raise NotImplementedError("Must be implemented in child class")

    @abc.abstractmethod
    def wait_for_usr_feedback(self) -> None:
        raise NotImplementedError("Must be implemented in child class")


class PlugInAdsAcActSrv(ManipulationActionServer):

    def __init__(self, name: str, state_machine: ManipulationStateMachine) -> None:
        super().__init__(name, state_machine)
        self.act_srv = actionlib.SimpleActionServer(name=self.name,
                                                    ActionSpec=PlugInAdsAcAction,
                                                    execute_cb=self.action_callback,
                                                    auto_start=False)
        self.act_srv.start()

    def action_callback(self, goal: PlugInAdsAcGoal) -> None:
        rospy.loginfo(f"Launch the plug-in process to the adapter station with AC plug")
        result_msg = PlugInAdsAcResult()
        try:
            ud = UserData()
            ud.job = self.name
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
        feedback = PlugInAdsAcFeedback()
        feedback.status = "wait_for_user"
        self.action_server.publish_feedback(feedback)


class ActSrvFactory:

    def __init__(self) -> None:
        self._selection: dict[str, Type[ManipulationActionServer]] = {}

    def register(self, name: str, act_srv: Type[ManipulationActionServer]) -> None:
        self._selection[name] = act_srv

    def create(self, name: str, state_machine: ManipulationStateMachine) -> ManipulationActionServer:
        act_srv_type = self._selection.get(name)
        if act_srv_type is None:
            raise KeyError(f"Unknown or unregistered ManipulationActionServer '{name}'"
                           f"Available server are: {list(self._selection.keys())}")
        return act_srv_type(state_machine=state_machine)


# Register all manipulation action servers
manipulation_action_server = ActSrvFactory()
manipulation_action_server.register('plug_in_ads_ac', PlugInAdsAcActSrv)
