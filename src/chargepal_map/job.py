from __future__ import annotations
# libs
from enum import Enum, auto


class Job:

    class Mode(Enum):
        PROGRESS = auto()
        RECOVER = auto()
        RETRY = auto()
        STOP = auto()

    class ID:
        move_home_arm = 'move_home_arm'
        free_drive_arm = 'free_drive_arm'

        marker_socket_calib_ads = 'marker_to_socket_calibration_ads'
        marker_socket_calib_bcs = 'marker_to_socket_calibration_bcs'

        plug_in_ads_ac = 'plug_in_ads_ac'
        plug_in_ads_dc = 'plug_in_ads_dc'
        plug_in_bcs_ac = 'plug_in_bcs_ac'
        plug_in_dsk_dm = 'plug_in_dsk_dm'

        plug_out_ads_ac = 'plug_out_ads_ac'
        plug_out_ads_dc = 'plug_out_ads_dc'
        plug_out_bcs_ac = 'plug_out_bcs_ac'
        plug_out_dsk_dm = 'plug_out_dsk_dm'

    def __init__(self, job_id: str) -> None:
        self._id = job_id
        self._mode = Job.Mode.STOP
        self.retry_count = 0
        if not self._is_valid():
            raise ValueError(f"Given job id is invalid: {self._id}")

    def __str__(self) -> str:
        return f"{self._id} - {self._mode.name}"

    def __repr__(self) -> str:
        class_name = type(self).__name__
        return f"{class_name}(id={self._id!r}, mode={self._mode!r})"

    def _is_valid(self) -> bool:
        """ Check if given job name is valid or not
        
        Returns:
            True if valid - False otherwise
        """
        valid = self._id in [
            Job.ID.move_home_arm,
            Job.ID.free_drive_arm,

            Job.ID.plug_in_ads_ac,
            Job.ID.plug_in_ads_dc,
            Job.ID.plug_in_ads_ac,
            Job.ID.plug_in_dsk_dm,
            
            Job.ID.plug_out_ads_ac,
            Job.ID.plug_out_ads_dc,
            Job.ID.plug_out_bcs_ac,
            Job.ID.plug_in_dsk_dm,
            ]
        return valid

    def start(self) -> None:
        self.retry_count = 0
        self._mode = Job.Mode.PROGRESS

    def in_progress_mode(self) -> bool:
        return self._mode == Job.Mode.PROGRESS

    def retry(self) -> None:
        self.retry_count += 1
        self._mode = Job.Mode.RETRY

    def in_retry_mode(self) -> bool:
        return self._mode == Job.Mode.RETRY

    def recover(self) -> None:
        self._mode = Job.Mode.RECOVER

    def in_recover_mode(self) -> bool:
        return self._mode == Job.Mode.RECOVER

    def stop(self) -> None:
        self._mode = Job.Mode.STOP

    def in_stop_mode(self) -> bool:
        return self._mode == Job.Mode.STOP

    def get_id(self) -> str:
        return self._id
    
    def get_plug_type(self) -> str:
        if self.is_part_of_type2_female():
            plug_type = 'type2_female'
        elif self.is_part_of_type2_male():
            plug_type = 'type2_male'
        elif self.is_part_of_ccs_female():
            plug_type = 'ccs_female'
        else:
            raise RuntimeError(f"Job ID {self._id} is not supposed to have a plug type")
        return plug_type

    def is_part_of_plug_in(self) -> bool:
        retval = self._id in [
            Job.ID.plug_in_ads_ac, 
            Job.ID.plug_in_ads_dc, 
            Job.ID.plug_in_bcs_ac, 
            Job.ID.plug_in_dsk_dm
            ]
        return retval

    def is_part_of_plug_out(self) -> bool:
        retval = self._id in [
            Job.ID.plug_out_ads_ac,
            Job.ID.plug_out_ads_dc,
            Job.ID.plug_out_bcs_ac,
            Job.ID.plug_out_dsk_dm
        ]
        return retval

    def is_part_of_workspace_left(self) -> bool:
        retval = self._id in [
            Job.ID.move_home_arm,
            Job.ID.free_drive_arm,
            Job.ID.plug_in_ads_ac,
            Job.ID.plug_in_bcs_ac,
            Job.ID.plug_in_dsk_dm,
            Job.ID.plug_out_ads_ac,
            Job.ID.plug_out_bcs_ac,
            Job.ID.plug_out_dsk_dm
        ]
        return retval

    def is_part_of_workspace_right(self) -> bool:
        retval = self._id in [
            Job.ID.move_home_arm,
            Job.ID.free_drive_arm,
            Job.ID.plug_in_ads_dc,
            Job.ID.plug_out_ads_dc
        ]
        return retval

    def is_part_of_type2_female(self) -> bool:
        retval = self._id in [
            Job.ID.plug_in_ads_ac,
            Job.ID.plug_out_ads_ac,
            Job.ID.plug_in_dsk_dm,
            Job.ID.plug_out_dsk_dm
        ]
        return retval

    def is_part_of_type2_male(self) -> bool:
        retval = self._id in [
            Job.ID.plug_in_bcs_ac,
            Job.ID.plug_out_bcs_ac
        ]
        return retval
    
    def is_part_of_ccs_female(self) -> bool:
        retval = self._id in [
            Job.ID.plug_in_ads_dc, 
            Job.ID.plug_out_ads_dc
        ]
        return retval
