import rospy
import PySimpleGUI as sg


class UserInterface:

    sg.theme('DarkAmber')   # Add a touch of color

    layout = [
        [sg.Button('Continue', size=(32, 9)), sg.Button('Stop', size=(32, 9))],
    ]

    def __init__(self) -> None:
        # Create the Window
        self.window = sg.Window('State Machine UI', self.layout, margins=(30, 30))
        
        rospy.wait_for_service('stop_process')
        rospy.wait_for_service('continue_process')


    def run(self) -> None:
        while True:
            event, values = self.window.read()
            if event == sg.WIN_CLOSED:
                break
            elif event == 'Continue':
                print('user continue')
            elif event == 'Stop':
                print('user stop')
        self.window.close()


if __name__ == '__main__':
    rospy.init_node('state_machine_user_interface')
    rospy.loginfo(f"Starting user interface to control the state machine process.")
    ui = UserInterface()
    rospy.loginfo(f"Ready to submit user requests.")
    ui.run()
