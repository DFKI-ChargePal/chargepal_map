from __future__ import annotations


class JobNames:

    plug_in_ads_ac = 'plug_in_ads_ac'
    plug_in_ads_dc = 'plug_in_ads_dc'
    plug_in_bcs_ac = 'plug_in_bcs_ac'

    plug_out_ads_ac = 'plug_out_ads_ac'
    plug_out_ads_dc = 'plug_out_ads_dc'
    plug_out_bcs_ac = 'plug_out_bcs_ac'

    def is_valid(job_name: str) -> bool:
        """ Check if given job name is valid or not
        
        Returns:
            True if valid - False otherwise
        """
        if job_name in [
            JobNames.plug_in_ads_ac, 
            JobNames.plug_in_ads_dc, 
            JobNames.plug_in_ads_ac, 
            
            JobNames.plug_out_ads_ac,
            JobNames.plug_out_ads_dc,
            JobNames.plug_out_bcs_ac,
            ]:
            valid = True
        else:
            valid = False
        return valid


    def plug_in() -> list[str]:
        """ Get all jobs which refer to the plug-in process

        Returns:
            List with valid job names
        """
        return [JobNames.plug_in_ads_ac, JobNames.plug_in_ads_dc, JobNames.plug_in_bcs_ac]
    
    def plug_out() -> list[str]:
        """ Get all jobs which refer to the plug-out process

        Returns:
            List with valid job names
        """
        return [JobNames.plug_out_ads_ac, JobNames.plug_out_ads_dc, JobNames.plug_out_bcs_ac]

    def workspace_left() -> list[str]:
        """ Get all jobs which have to be executed in the left workspace

        Returns:
            List with valid job names
        """
        return [JobNames.plug_in_ads_ac, JobNames.plug_in_bcs_ac, JobNames.plug_out_ads_ac, JobNames.plug_out_bcs_ac]
    
    def workspace_right() -> list[str]:
        """ Get all jobs which have to be executed in the right workspace

        Returns:
            List with valid job names
        """
        return [JobNames.plug_in_ads_dc, JobNames.plug_out_ads_dc]

    def type2_female() -> list[str]:
        """ Get all jobs which uses the 'Type2' (AC) plug with female inlet

        Returns:
            List with valid job names
        """
        return [JobNames.plug_in_ads_ac, JobNames.plug_out_ads_ac]
    
    def type2_male() -> list[str]:
        """ Get all jobs which uses the 'Type2' (AC) plug with male inlet

        Returns:
            List with valid job names
        """
        return [JobNames.plug_in_bcs_ac, JobNames.plug_out_bcs_ac]
    
    def ccs_female() -> list[str]:
        """ Get all jobs which uses the 'CCS' (DC) plug with female inlet

        Returns:
            List with valid job names
        """
        return [JobNames.plug_in_ads_dc, JobNames.plug_out_ads_dc]


job_ids = JobNames()
