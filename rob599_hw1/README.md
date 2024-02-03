## Running Marcus Rosette's Homework 1 Code

1. First start up the fetch-gazebo environment 

```bash
roslaunch rob599_basic fetch_world.launch
```

2. I then needed to disable gravity in the wrist of the fetch arm in the gazebo environment. This ensures the arm does not get in the way of the lidar sensor

3. Start Rviz to vizualize each of the implemented topics
    Items to include in Rviz
    a. RobotModel
    b. LaserScan - \filtered_scan
    c. MarkerArray - \rviz_markers
    d. Marker - \wall_angle_maker

4. Launch wall_stare.launch
```bash
roslaunch rob599_hw1 wall_stare.launch
```
    a. This brings up the robot_approach node that has fetch approach a wall until a stopping distance threshold is met
    b. It also brings up the filtered_scan node that filters incomming laserscan messages to only view scans that are directly in front of fetch
    c. It also brings up the wall_angle node that estimates the angle of the wall relative to fetch

5. To test the stopping_distance service, run
```bash
rosservice call /stopping_distance "stopping_distance: 0.75"
```
This will change the default 1m stopping distance to 0.75m

6. To test the action interface, run
```bash
rosrun rob599_hw1 approach_action_client.py 
```
This will set a new distance threshold as defined by approach_action_client.py, provide feedback on its progress in approaching the wall, and indicate if fetch made it to the new distance threshold