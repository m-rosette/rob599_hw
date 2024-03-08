from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    rob599_hw3_map = '/home/marcus/classes/rob599/ros2_ws/src/rob599_hw3/resource/map.yaml'
    
    return LaunchDescription([
        # Set environment variable
        SetEnvironmentVariable(name='LDS_MODEL', value='LDS-01'),
		SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger'),

		# Start the Turtlebot 3 simulation in Gazebo.
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([
				os.path.join(
					get_package_share_directory('turtlebot3_gazebo'),
					'launch/turtlebot3_house.launch.py'
				)
			]),
			launch_arguments={
				'x_pose': '-1.5',
				'y_pose': '1.5'
			}.items()
		),

        # Launch Navigation Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('turtlebot3_navigation2'),
                    'launch/navigation2.launch.py'
                )
            ]),
            launch_arguments={
                'use_sim_time': 'True',
                'map': rob599_hw3_map,
            }.items()
        ),

	])