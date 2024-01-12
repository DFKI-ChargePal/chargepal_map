# global
from actionlib_msgs.msg import GoalStatus

# local
from chargepal_map.ui import ProcessStatus

# typing
from actionlib import SimpleActionClient


class ActionClient:

    def __init__(self, sac: SimpleActionClient, goal_type: type) -> None:
        self.sac = sac
        self.goal = goal_type
        self.active = False

    def start(self, state: ProcessStatus) -> ProcessStatus:
        self.active = True
        if state is ProcessStatus.READY:
            self.sac.send_goal(self.goal())
            new_state = ProcessStatus.RUN
        else:
            new_state = state
        return new_state

    def update(self, state: ProcessStatus) -> ProcessStatus:
        # print(self.sac.get_result())
        new_state = state
        if self.active:
            ac_state = self.sac.get_state()
            if ac_state == GoalStatus.ACTIVE and not state == ProcessStatus.WAIT_FOR_USR:
                new_state = ProcessStatus.RUN
            elif ac_state == GoalStatus.SUCCEEDED:
                new_state = self._fin(state)
        return new_state

    def _fin(self, state: ProcessStatus) -> ProcessStatus:
        self.active = False
        new_state = ProcessStatus.READY
        return new_state
