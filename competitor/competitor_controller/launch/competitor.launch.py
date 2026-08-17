"""Launch the competitor's controller.

use_sim_time is on because the simulator drives everything off Gazebo's clock,
which reaches this container over the domain bridge as /clock. With it off the
controller's timers run on wall time and its derivative term is wrong whenever
the real-time factor is not 1.0.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'heave_sign',
            default_value='1.0',
            description=(
                'Flip to -1.0 if the vehicle rises when commanded to descend. '
                'simulator_bridge negates heave on its way to Gazebo and '
                'AuvState z is already sign-flipped, so this is settled by '
                'observation, not by reading the code.'
            ),
        ),
        Node(
            package='competitor_controller',
            executable='basic_controller',
            name='basic_controller',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                # Without the explicit value_type this arrives as the string
                # "1.0" and the node's double parameter rejects it.
                'heave_sign': ParameterValue(
                    LaunchConfiguration('heave_sign'), value_type=float),
            }],
        ),
    ])
