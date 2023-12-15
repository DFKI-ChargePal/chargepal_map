import re


def state_name(obj: object) -> str:
    # Split object name at uppercase letters
    upper_split = re.findall('[A-Z][^A-Z]*', obj.__name__)
    # Make all splits uppercase and connect them by _
    name = "_".join(upper_split).upper()
    return name