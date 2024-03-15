class Outcomes:

    # Outcome mapping to avoid typos
    start = 'start'
    stop = 'stop'

    arm_in_ws_ls = 'arm_in_work_space_left_side'
    arm_in_ws_rs = 'arm_in_work_space_right_side'

    socket_pre_obs = 'socket_pre_observed'
    socket_obs = 'socket_observed'

    plug_pre_obs = 'plug_pre_observed'
    plug_obs = 'plug_observed'

    plug_pre_connect = 'plug_pre_connect'
    plug_connect = 'plug_connect'

    plug_pre_attached = 'plug_pre_attached'
    plug_attached = 'plug_attached'
    
    plug_removed = 'plug_removed'
    plug_released = 'plug_released'



# Common process outcomes
class Common:

    # Outcome mapping
    stop = 'stop'


# Connect battery with car
class ConnectToCarTwist:

    # Outcome mapping
    arm_in_bat_pre_obs = 'arm_in_battery_pre_observation'
    arm_in_bat_post_obs = 'arm_in_battery_post_observation'
    arm_in_bat_pre_connect = 'arm_in_battery_pre_connection'
    plug_in_bat_connect = 'plug_in_battery_connection'
    plug_in_bat_post_connect = 'plug_in_battery_post_connection'
    plug_in_car_pre_obs = 'plug_in_car_pre_observation'
    plug_in_car_post_obs = 'plug_in_car_post_observation'
    plug_in_car_pre_connect = 'plug_in_car_pre_connection'
    plug_in_car_connect = 'plug_in_car_connection'
    arm_in_car_post_connect = 'arm_in_car_post_connection'
    arm_in_driving_pose = 'arm_in_driving_pose'


class ConnectToCarElectric:

    # Outcome mapping
    arm_in_bat_pre_obs = 'arm_in_battery_pre_observation'
    arm_in_bat_post_obs = 'arm_in_battery_post_observation'
    arm_in_bat_pre_connect = 'arm_in_battery_pre_connection'
    plug_in_bat_connect = 'plug_in_battery_connection'
    plug_in_bat_post_connect = 'plug_in_battery_post_connection'
    plug_in_car_pre_obs = 'plug_in_car_pre_observation'
    plug_in_car_post_obs = 'plug_in_car_post_observation'
    plug_in_car_pre_connect = 'plug_in_car_pre_connection'
    plug_in_car_connect = 'plug_in_car_connection'
    arm_in_car_post_connect = 'arm_in_car_post_connection'
    arm_in_driving_pose = 'arm_in_driving_pose'


# Disconnect battery with adapter station
class DisconnectFromCarTwist:

    # Outcome mapping
    arm_in_car_pre_obs = 'arm_in_car_pre_observation'
    arm_in_car_post_obs = 'arm_in_car_post_observation'
    arm_in_car_pre_connect = 'arm_in_car_pre_connection'
    plug_in_car_connect = 'plug_in_car_connection'
    plug_in_car_post_connect = 'plug_in_car_post_connection'
    plug_in_bat_pre_obs = 'plug_in_battery_pre_observation'
    plug_in_bat_post_obs = 'plug_in_battery_post_observation'
    plug_in_bat_pre_connect = 'plug_in_battery_pre_connection'
    plug_in_bat_connect = 'plug_in_battery_connection'
    arm_in_bat_post_connect = 'arm_in_battery_post_connection'
    arm_in_driving_pose = 'arm_in_driving_pose'



class DisconnectFromCarElectric:

    # Outcome mapping
    arm_in_car_pre_obs = 'arm_in_car_pre_observation'
    arm_in_car_post_obs = 'arm_in_car_post_observation'
    arm_in_car_pre_connect = 'arm_in_car_pre_connection'
    plug_in_car_connect = 'plug_in_car_connection'
    plug_in_car_post_connect = 'plug_in_car_post_connection'
    plug_in_bat_pre_obs = 'plug_in_battery_pre_observation'
    plug_in_bat_post_obs = 'plug_in_battery_post_observation'
    plug_in_bat_pre_connect = 'plug_in_battery_pre_connection'
    plug_in_bat_connect = 'plug_in_battery_connection'
    arm_in_bat_post_connect = 'arm_in_battery_post_connection'
    arm_in_driving_pose = 'arm_in_driving_pose'
