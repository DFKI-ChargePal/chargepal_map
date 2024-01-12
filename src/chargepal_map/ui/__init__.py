from enum import Enum, auto


class UserSrv:
    continue_process = 'continue_process'
    stop_process = 'stop_process'


class ProcessStatus(Enum):
    RUN = auto()
    STOP = auto()
    READY = auto()
    WAIT_FOR_USR = auto()
