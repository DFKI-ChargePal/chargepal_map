from __future__ import annotations
import re
import smach


class StateMachineError(Exception):

    def __init__(self, *args: object) -> None:
        super().__init__(*args)


def state_name(obj: type) -> str:
    # Split object name at uppercase letters
    upper_split = re.findall('[A-Z][^A-Z]*', obj.__name__)
    # Make all splits uppercase and connect them by _
    name = "_".join(upper_split).upper()
    return name


def silent_smach() -> None:
    def suppress_msg(msg: str) -> None:
        pass
    smach.set_loggers(info=suppress_msg, warn=smach.logwarn, debug=suppress_msg, error=smach.logerr)


def state_header(obj: type) -> str:
    return f"\n{5*'-'}   State: {state_name(obj)}   {5*'-'}"


def state_footer(obj: type) -> str:
    return f"{(len(state_header(obj)) - 1) * '-'}"
