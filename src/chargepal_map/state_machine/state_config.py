from __future__ import annotations

# global
import re
import copy
import yaml

# typing
from typing import Any


class StateConfig:

    def __init__(self, obj: type, config: dict[str, dict[str, Any]]) -> None:
        # Get common configuration
        self.data: dict[str, Any] = {}
        self.data['step_by_user'] = config['step_by_user']
        self.data['detector'] = copy.deepcopy(config['detector']['files'])
        # Split object name at uppercase letters
        upper_split = re.findall('[A-Z][^A-Z]*', obj.__name__)
        # Make all splits lowercase and connect them by _
        self.name = "_".join(upper_split).lower()
        # Read configuration
        fp_config = config['states']['files'][self.name]
        with fp_config.open('r') as fp:
            try:
                state_config_dict: dict[str, Any] = yaml.safe_load(fp)
            except Exception as e:
                raise RuntimeError(f"Error while reading {fp_config} configuration with error msg: {e}")
        if state_config_dict is None:
            state_config_dict = {}
        self.data.update(state_config_dict)

    def dump(self) -> str:
        _data = {
            self.name: self.data
        }
        return yaml.dump(_data)
