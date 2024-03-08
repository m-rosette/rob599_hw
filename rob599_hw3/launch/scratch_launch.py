import os
import launch
import launch_ros.actions
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # Set the path to the Turtlebot3 simulation package
    turtlebot3_gazebo_pkg = os.getenv('TURTLEBOT3_GAZEBO_PACKAGE', 'turtlebot3_gazebo')

    # Set the path to the house world file
    house_world = os.path.join(turtlebot3_gazebo_pkg, 'worlds', 'house.world')

    return launch.LaunchDescription([
        # Set the TurtleBot3 model environment variable
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger'),

        # Launch Gazebo simulation with the house world
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            output='screen',
            arguments=[house_world]
            ),

        # Launch Turtlebot3 robot node
        launch_ros.actions.Node(
            package='turtlebot3_node',
            executable='turtlebot3_node',
            name='turtlebot3_node',
            output='screen'
        ),

        # Launch SLAM node
        launch_ros.actions.Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'scan_downsample': 10,
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'output_frame_id': 'map',
                'publish_period_sec': 1.0,
                'enable_loop_closure': False,
                'enable_map_update': True,
                'use_pose_estimation': False
            }]
        ),

        # Launch Navigation Stack
        launch_ros.actions.Node(
            package='nav2_bringup',
            executable='nav2_bringup',
            name='nav2_bringup',
            output='screen'
        ),

    ])

