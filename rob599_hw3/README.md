# ROB599 Homework 3

## Marcus Rosette

This homework interfaces with the turtlebot3 and nav2 packages. Generating a SLAM map and navigating the turtlebot3 to varying points in the turtlebot3 gazebo house environment. The generated slam map is then used as a basis for maneuvering the robot through the house.


### Package installation

To interact with this package first follow the [turtlebot3 installation instructions](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

For the purposes of this assignment, the **burger** turtlebot3 model will be used

Clone this repo into your ros2_ws by running
```bash
cd ros2_ws/src/
git clone $this-repo$
cd ~/ros2_ws/
colcon build
```


### Generating a SLAM map

To start generating a SLAM map of the turtlebot3 house environment, launch
```bash
ros2 launch rob599_hw3 turtlebot3_slam_launch.py
```

Generate a more detailed map by having the robot traverse the environment by teleoperating around the house
```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard 
```

The generated map can be saved by running
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```


### Interacting with a generated map in rviz2

1. To load the turtlebot3 in the generated rviz2 map and simulated house world, launch
```bash
ros2 launch rob599_hw3 turtlebot3_nav_launch.py
```

2. Provide a *2D Pose Estimate* for the nav-stack by clicking on the pre-generated coordinate frame

3. To bagin saving poses, start the places node by running
```bash
ros2 run rob599_hw3 places 
```

4. To enable the visualization of presaved positions, in the *Displays* panel, select *Add->By topic->/saved_positions->Marker* 

5. New markers visualizing the turtlebot3 pose can be places by running
```bash
ros2 service call /memorize_position rob599_hw3_msgs/srv/MemorizePosition "{position_name: '$some-position-name$'}"
```

6. To clear the visualized markers/poses, run
```bash
ros2 service call /clear_positions rob599_hw3_msgs/srv/ClearPositions 
```

7. Any visualized markers/poses can be saved as a *.yaml* file to the *resource/* directory by running 
```bash
ros2 service call /save_places rob599_hw3_msgs/srv/Save "{filename: 'explore_house'}"
```

8. Saved markers/poses can be loaded into the environment by running
```bash
ros2 service call /load_places rob599_hw3_msgs/srv/Load "{filename: 'explore_house'}"
```

9. To navigate the turtlebot3 to one of the saved poses (table or other poses in the saved .yaml) run
```bash
ros2 action send_goal /go_to rob599_hw3_msgs/action/GoTo "{position_name: 'table'}"
```
OR 
```bash
ros2 run rob599_hw3 goto_client table
```

10. To have the turtlebot3 navigate (patrol) to each presaved poses, run
```bash
ros2 action send_goal /patrol rob599_hw3_msgs/action/Patrol "{}"
```

11. With an existing action running (either step 9 or 10), the turtlebot3 can check the front door (if the position exists), preempting the action
```bash
ros2 service call /knock_knock rob599_hw3_msgs/srv/KnockKnock "{}"
```
