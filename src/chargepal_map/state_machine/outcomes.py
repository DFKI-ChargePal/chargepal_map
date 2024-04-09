
class Outcomes:

    # Outcome mapping to avoid typos
    stop = 'stop'
    start = 'start'
    failure = 'failure'
    completed = 'completed'

    arm_in_wrong_ws = 'arm_in_wrong_workspace'
    arm_ready_do = 'arm_ready_do_observation'
    arm_ready_no = 'arm_ready_no_observation'

    socket_pre_obs = 'socket_pre_observed'
    socket_obs = 'socket_observed'

    plug_pre_obs = 'plug_pre_observed'
    plug_obs = 'plug_observed'

    plug_pre_connected = 'plug_pre_connect'
    plug_connected = 'plug_connect'

    plug_pre_attached = 'plug_pre_attached'
    plug_attached = 'plug_attached'
    
    plug_removed_do = 'plug_removed_do_observation'
    plug_removed_no = 'plug_removed_no_observation'
    plug_released = 'plug_released'


out = Outcomes()
