# Common process outcomes
class Common:

    # Outcome mapping
    stop = 'stop'


# Connect battery with car
class ConnectToCar:

    # Outcome mapping
    arm_in_bat_obs = 'arm_in_battery_observation'
    arm_in_bat_pre_connect = 'arm_in_battery_pre_connection'
    plug_in_bat_connect = 'plug_in_battery_connection'
    plug_in_bat_post_connect = 'plug_in_battery_post_connection'
    plug_in_car_obs = 'plug_in_car_observation'
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
class DisconnectFromCar:

    # Outcome mapping
    arm_in_car_obs = 'arm_in_car_observation'
    arm_in_car_pre_connect = 'arm_in_car_pre_connection'
    plug_in_car_connect = 'plug_in_car_connection'
    plug_in_car_post_connect = 'plug_in_car_post_connection'
    plug_in_bat_obs = 'plug_in_battery_observation'
    plug_in_bat_pre_connect = 'plug_in_battery_pre_connection'
    plug_in_bat_connect = 'plug_in_battery_connection'
    arm_in_bat_post_connect = 'arm_in_battery_post_connection'
    arm_in_driving_pose = 'arm_in_driving_pose'


class DisconnectFromCar:

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
