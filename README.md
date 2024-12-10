# ChargePal Manipulation Action Processor (MAP)

ROS package with a state machine using SMACH to run the different manipulation jobs.

## State Machine Structure

![StateMachine](./doc/StateMachine.svg)


## Getting started

1) Create a ROS workspace with the following packages

- [chargepal_actions](https://github.com/DFKI-ChargePal/chargepal_actions)

- [chargepal_services](https://github.com/DFKI-ChargePal/chargepal_services)

- [chargepal_map](https://github.com/DFKI-ChargePal/chargepal_map)

2) Clone configuration repository into `chargepal_map` folder

```commandline
    # Navigate to ROS package
    roscd chargepal_map
    git clone https://github.com/DFKI-ChargePal/chargepal_configuration.git config
    
    # Check out a proper configuration branch
    cd config
    git switch map/xxx/xxx
```

3) Install UR-Pilot

```commandline
    pip install git+https://github.com/DFKI-ChargePal/chargepal_ur_pilot.git
```

4) Run roslaunch script from your sourced terminal

```commandline
    roslaunch chargepal_map action_server.launch
```

5) The state machine can be triggered via action calls
```commandline
    # Get available action topics
    rostopic list
```
