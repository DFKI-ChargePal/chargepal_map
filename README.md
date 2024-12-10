# ChargePal Manipulation Action Processor (MAP)

ROS package with a state machine using SMACH to run the different manipulation jobs.

## State Machine Structure

![StateMachine](./doc/StateMachine.svg)


## Getting started

1) Create a ROS workspace with the following dependencies
```
[chargepal_actions]()
```

1) Install the package in your catkin workspace.

----

2) Run roslaunch script from your sourced terminal

```commandline
    roslaunch chargepal_map action_server.launch
```
----
