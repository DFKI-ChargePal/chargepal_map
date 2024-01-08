from __future__ import annotations

# global
import re

# typing
from typing import Any


class StateConfig:

    def __init__(self, obj: type, config: dict[str, dict[str, Any]]) -> None:
        
        # Try to get common configuration
        self.common = config.get('common')
        if self.common is None:
            self.common = {'common': None}
        
        # Split object name at uppercase letters
        upper_split = re.findall('[A-Z][^A-Z]*', obj.__name__)
        # Make all splits lowercase and connect them by _
        name = "_".join(upper_split).lower()
        # Try to get configuration with class name
        self.state = config.get(name)
        if self.state is None:
            self.state = {name: None}
