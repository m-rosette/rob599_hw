# ROB599 Homework 2

## Marcus Rosette

The first part of this homework has the goal of generating Twist messages, capping them to parameter set maximum values, checking the number of messages that exceed those maximums, and publishing the capped Twist messages

To run this functionality, run

```bash
ros2 launch rob599_hw2 speed_limit_launch.py
```

This launches three nodes:
    1. **twist_gen**: publishes random Twist values on the *speed_in* topic every 1 second, generating random linear velocities between -5 and 5m/s, and angular velocities between -2.5 and 2.5rads/s (completely arbitrary)
    2. **velocity_limiter**: subscribes to the *speed_in*, capping the velocities based on the set parameters *linear_max* and *angular_max* which default to 4m/s and 2rads/s, respectively. The capped Twist messages are then published on the *speed_out* topic. There is also an optional watchdog (changable bool under parameter *with_watchdog*) that checks if Twist messages on *speed_in* were published within the *watchdog_period* set parameter. If no message was recieved, a zero Twist message is published on the *speed_out* topic
    3. **velocity_checker**: checks to see how many generated Twist messages were over the set parameter limits; *linear_max_check* and *angular_max_check*. 

A service call can be made to **velocity_limiter** that enables the brakes on a robot to be turned on. Or in this simulated case, a zero Twist message to be sent. A service client named **braking_service_client** handles: apply brakes = "1" or turn off brakes "0". To execute this, run 

```bash
ros2 run rob599_hw2 braking_service_client $0 or 1$
```

This homework also emulates a NASA rocket launch, which can be started by running,

```bash
ros2 run rob599_hw2 nasa_action_server
```

This starts the action and waits for a goal to be sent. The rocket can be launched by running the action client, which has a default countdown time of 10 seconds. Launch the rocket by running,

```bash
ros2 run rob599_hw2 nasa_action_client
```

If canceling the launch is desired, this can be done by setting a cancelation bool (True for launch cancelation, else continue) parameter in a separate terminal,

```bash
ros2 param set /nasa_action_client abort_launch True
```