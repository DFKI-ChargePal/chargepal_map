import rospy
import actionlib
import PySimpleGUI as sg
from enum import Enum, auto
from std_srvs.srv import Empty

# actions
from chargepal_actions.msg import (
    ConnectPlugToCarAction,
    ConnectPlugToCarActionGoal,
    DisconnectPlugFromCarAction,
    DisconnectPlugFromCarActionGoal,
)
from actionlib_msgs.msg import GoalStatus


class ProcessStatus(Enum):
    RUN = auto()
    STOP = auto()
    READY = auto()
    WAIT_FOR_USR = auto()


class UserInterface:

    sg.theme('DarkAmber')   # Add a touch of color

    layout = [
        [sg.Text('Manipulation action processes:'), sg.Text(' ', key='-PROCESS-')],
        [sg.Button('Connect To Car', size=(32, 9), key='-CONNECT_TO_CAR-'), 
         sg.Button('Disconnect From Car', size=(32, 9), key='-DISCONNECT_FROM_CAR-')],
        [sg.Text('Process status: '), sg.Text('READY', text_color='blue', key='-STATUS-')],
        [sg.Text('    ')],
        [sg.Text('User commands: ')],
        [sg.Button('Continue', size=(32, 7)), sg.Button('Stop', size=(32, 7))],
    ]

    def __init__(self) -> None:
        # Create the Window
        self.window = sg.Window('State Machine UI', self.layout, margins=(30, 30))
        # Initialize action clients
        self._ctc_ac = actionlib.SimpleActionClient('connect_to_car', ConnectPlugToCarAction)
        self._dfc_ac = actionlib.SimpleActionClient('disconnect_from_car', DisconnectPlugFromCarAction)
        rospy.wait_for_service('stop_process')
        rospy.wait_for_service('continue_process')
        self.state = ProcessStatus.READY


    def run(self) -> None:
        while True:
            event, values = self.window.read(timeout=500)
            if event == sg.WIN_CLOSED:
                break
            if event == 'Continue':
                try:
                    continue_process = rospy.ServiceProxy('continue_process', Empty)
                    continue_process()
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call to continue the process failed. {e}")
            elif event == 'Stop':
                try:
                    stop_process = rospy.ServiceProxy('stop_process', Empty)
                    stop_process()
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call to stop process failed. {e}")
                self.state = ProcessStatus.STOP
            
            ctc_ac_state = self._ctc_ac.get_state()
            dfc_ac_state = self._dfc_ac.get_state()
            # rospy.loginfo(f"Action client CTC state: {self._ctc_ac.get_state()}")
            # rospy.loginfo(f"Action client DFC state: {self._dfc_ac.get_state()}")
            if ctc_ac_state == GoalStatus.ACTIVE or dfc_ac_state == GoalStatus.ACTIVE:
                self.state = ProcessStatus.RUN
            elif (ctc_ac_state == GoalStatus.SUCCEEDED or ctc_ac_state == GoalStatus.LOST) and (dfc_ac_state == GoalStatus.SUCCEEDED or dfc_ac_state == GoalStatus.LOST):
                self.state = ProcessStatus.READY
            else:
                self.state = ProcessStatus.STOP
            
            if self.state == ProcessStatus.READY and event == '-CONNECT_TO_CAR-':
                # Send goal to action server
                goal = ConnectPlugToCarActionGoal()
                self._ctc_ac.send_goal(goal=goal)
                self.window['-PROCESS-'].update('Connect To Car')
                self.state = ProcessStatus.RUN
            elif self.state == ProcessStatus.READY and event == '-DISCONNECT_FROM_CAR-':
                # Sent goal to action server
                goal = DisconnectPlugFromCarActionGoal()
                self._dfc_ac.send_goal(goal=goal)
                self.window['-PROCESS-'].update('Disconnect From Car')
                self.state = ProcessStatus.RUN

            if self.state == ProcessStatus.RUN:
                self.window['-STATUS-'].update(value='RUN', text_color='green')
            elif self.state == ProcessStatus.STOP:
                self.window['-STATUS-'].update(value='STOP', text_color='red')
            elif self.state == ProcessStatus.READY:
                self.window['-STATUS-'].update(value='READY', text_color='blue')
            else:
                self.window['-STATUS-'].update(value='')
        
        self.window.close()


if __name__ == '__main__':
    rospy.init_node('state_machine_user_interface')
    rospy.loginfo(f"Starting user interface to control the state machine process.")
    ui = UserInterface()
    rospy.loginfo(f"Ready to submit user requests.")
    ui.run()
