import launch
import launch_ros.actions

def generate_launch_description():
    # Define parameters for the twist_gen node
    velocity_limiter_params = [
        {"linear_max": 4.0},
        {"angular_max": 2.0}
    ]

    # Define parameters for the velocity_limiter node
    velocity_checker_params = [
        {"linear_max_check": 4.0},
        {"angular_max_check": 2.0}
    ]

    return launch.LaunchDescription([
        # The Twist generator node
        launch_ros.actions.Node(
            package='rob599_hw2',
            executable='twist_gen',
            name='twist_generator',
        ),

        # The limiter node
        launch_ros.actions.Node(
            package='rob599_hw2',
            executable='velocity_limiter',
            name='velocity_limiter',
            parameters=velocity_limiter_params,
            ),

        # The checker node
        launch_ros.actions.Node(
            package='rob599_hw2',
            executable='velocity_checker',
            name='velocity_checker',
            parameters=velocity_checker_params,
            ),

        ])
