from __future__ import annotations

# global
import re
import copy

# typing
from typing import Any


class StateConfig:

    def __init__(self, obj: type, config: dict[str, dict[str, Any]]) -> None:
        
        # Try to get common configuration
        common_data = copy.deepcopy(config.get('common'))
        if common_data is None:
            common_data = {}
        # Set default values if not already exists
        common_data.setdefault('step_by_user', False)

        # Split object name at uppercase letters
        upper_split = re.findall('[A-Z][^A-Z]*', obj.__name__)
        # Make all splits lowercase and connect them by _
        name = "_".join(upper_split).lower()
        # Try to get configuration with class name
        state_data = copy.deepcopy(config.get(name))
        if state_data is None:
            state_data = {}
        # Set default values if not already exists
        # self.state.setdefault()

        # Set configuration data
        common_data.update(state_data)
        self.data = common_data
