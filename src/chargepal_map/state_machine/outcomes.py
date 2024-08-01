""" This file maps the outcome names to variables to avoid typos """

job_failed = 'job_failed'
job_stopped = 'job_stopped'
job_complete = 'job_complete'
job_incomplete = 'job_incomplete'

err_obs_retry = 'error_while_observing_retry'
err_obs_recover = 'error_while_observing_recover'

err_plug_in_stop = 'error_plug_in_stop'
err_plug_in_retry = 'error_plug_in_retry'
err_plug_in_recover = 'error_plug_in_recover'
err_plug_out_stop = 'error_plug_out_stop'
err_plug_out_retry = 'error_plug_out_retry'
err_plug_out_recover = 'error_plug_out_recover'

arm_in_wrong_ws = 'arm_in_wrong_workspace'
arm_ready_to_go = 'arm_ready_to_go'
arm_ready_to_free = 'arm_ready_to_free'
arm_ready_to_move_ls = 'arm_ready_to_move_within_left_workspace'
arm_ready_to_move_rs = 'arm_ready_to_move_within_right_workspace'

plug_obs = 'plug_observed'
socket_obs = 'socket_observed'
battery_plug_id_obs = 'battery_plug_id_observed'
battery_obs_plug_in = 'battery_observed_plug_in'
battery_obs_plug_out = 'battery_observed_plug_out'
periphery_plug_obs = 'periphery_plug_observed'
periphery_socket_obs = 'periphery_socket_observed'
periphery_plug_id_obs = 'periphery_plug_id_observed'

plug_pre_obs = 'plug_pre_observed'
socket_pre_obs = 'socket_pre_observed'
plug_id_pre_obs = 'plug_id_pre_observed'
battery_pre_obs = 'battery_pre_observed'
periphery_pre_obs = 'periphery_pre_observed'

plug_pre_pos = 'plug_pre_position'
socket_pre_pos = 'socket_pre_position'
recover_pre_pos = 'recover_pre_position'

plug_attached = 'plug_attached'
plug_released = 'plug_released'
plug_connected = 'plug_connected'
battery_plug_removed = 'battery_plug_removed'
periphery_plug_removed = 'periphery_plug_removed'
