from __future__ import annotations

# global
import rospy
import actionlib
import PySimpleGUI as sg

# local
from chargepal_map.ui import ProcessStatus, UserSrv
from chargepal_map.ui.action_client import ActionClient

# services
from chargepal_map.srv import User, UserRequest, UserResponse

# actions
from chargepal_actions.msg import (
    ConnectPlugToCarAction,
    ConnectPlugToCarActionGoal,
    DisconnectPlugFromCarAction,
    DisconnectPlugFromCarActionGoal,
)


class UserInterface:

    sg.theme('DarkAmber')   # Add a touch of color

    layout = [
        [sg.Text('Manipulation action processes:'), sg.Text(' ', key='-PROCESS-')],
        [sg.Button('Connect To Car', size=(32, 9), key='-CONNECT_TO_CAR-'), 
         sg.Button('Disconnect From Car', size=(32, 9), key='-DISCONNECT_FROM_CAR-')],
        [sg.Text('Process status: '), sg.Text('READY', text_color='blue', key='-STATUS-')],
        [sg.Text('    ')],
        [sg.Text('User commands: ')],
        [sg.Button('Continue', size=(32, 7), disabled=True), sg.Button('Stop', size=(32, 7), disabled=True)],
    ]

    def __init__(self) -> None:
        # Create the Window
        self.window = sg.Window('State Machine UI', self.layout, margins=(30, 30))
        # Initialize action clients
        ctc_sac = actionlib.SimpleActionClient('connect_to_car', ConnectPlugToCarAction)
        dfc_sac = actionlib.SimpleActionClient('disconnect_from_car', DisconnectPlugFromCarAction)
        self.action_clients = {
            'connect_to_car': ActionClient(ctc_sac, ConnectPlugToCarActionGoal),
            'disconnect_from_car': ActionClient(dfc_sac, DisconnectPlugFromCarActionGoal)
        }
        # Declare user request service
        self._usr_srv = rospy.Service('user', User, self._handle_user_request)
        # Interactive values
        self.win_event = ''
        self._win_values = None
        self._state = ProcessStatus.READY

    @property
    def state(self) -> ProcessStatus:
        return self._state

    @state.setter
    def state(self, value: ProcessStatus) -> None:
        self._state = value
        self._update_state()

    def _update_state(self) -> None:
        if self.state == ProcessStatus.RUN:
            self.window['-STATUS-'].update(value='RUN', text_color='green')
        elif self.state == ProcessStatus.STOP:
            self.window['-STATUS-'].update(value='STOP', text_color='red')
        elif self.state == ProcessStatus.READY:
            self.window['-STATUS-'].update(value='READY', text_color='blue')
            self.window['-PROCESS-'].update(' ')
        elif self.state == ProcessStatus.WAIT_FOR_USR:
            self.window['-STATUS-'].update(value='WAIT FOR USER', text_color='orange')
        else:
            self.window['-STATUS-'].update(value='')

    def _handle_user_request(self, req: UserRequest) -> UserResponse:
        # Enable user interaction
        self.window['Continue'].update(disabled=False)
        self.window['Stop'].update(disabled=False)
        self.state = ProcessStatus.WAIT_FOR_USR
        res = UserResponse()
        while True:
            if self.win_event == 'Continue':
                res.user_action = UserSrv.continue_process
                self.state = ProcessStatus.RUN
                break
            elif self.win_event == 'Stop':
                res.user_action = UserSrv.stop_process
                self.state = ProcessStatus.STOP
                break
        # Enable user interaction
        self.window['Continue'].update(disabled=True)
        self.window['Stop'].update(disabled=True)
        return res

    def run(self) -> None:
        while True:
            self.win_event, self._win_values = self.window.read(timeout=500)
            if self.win_event == sg.WIN_CLOSED:
                break
            if self.win_event == '-CONNECT_TO_CAR-':
                # Start action server
                self.state = self.action_clients['connect_to_car'].start(self.state)
                self.window['-PROCESS-'].update('Connect To Car')
            elif self.win_event == '-DISCONNECT_FROM_CAR-':
                # Start action server
                self.state = self.action_clients['disconnect_from_car'].start(self.state)
                self.window['-PROCESS-'].update('Disconnect From Car')
                
            for ac in self.action_clients.values():
                self.state = ac.update(self.state)
        self.window.close()


class UserInterfaceFactory:

    @staticmethod
    def create() -> UserInterface:
        return UserInterface()


ui = UserInterfaceFactory()
