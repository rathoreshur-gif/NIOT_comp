"""Launch the gamepad teleop instead of the competition controller.

This is the demo entry point: `basic_controller` stays exactly as it was and is
not started here. Only one node may publish /controller/thruster_forces, so
running both at once makes the two fight and the vehicle does something
incoherent - which is why this is a separate launch file rather than an extra
node bolted onto competitor.launch.py.

    ros2 launch competitor_controller gamepad.launch.py
    ros2 launch competitor_controller gamepad.launch.py device:=/dev/input/js1

use_sim_time is on for the same reason it is in competitor.launch.py: the
simulator drives everything off Gazebo's clock, which arrives here as /clock
over the domain bridge. With it off the control loop runs on wall time and the
velocity loops are wrong whenever the real-time factor is not 1.0. The pad
itself is read on wall time regardless, so input never stalls with the clock.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/input/js0',
            description=(
                'Joystick device. The pad has to be passed into the container: '
                'docker-compose.demo.yml does that, and JOY_DEV overrides which '
                'one. `ls /dev/input/js*` on the host to find it.'
            ),
        ),
        DeclareLaunchArgument(
            'heave_sign',
            default_value='1.0',
            description=(
                'Flip to -1.0 if the vehicle rises when told to dive. Same '
                'escape hatch competitor.launch.py carries, for the same '
                'reason: the heave sign is settled by observation.'
            ),
        ),
        DeclareLaunchArgument(
            'cruise_speed',
            default_value='1.35',
            description=(
                'Speed in m/s at full stick with no trigger held. Reachable '
                'only because the demo raises max_thrust to 75 N; on the '
                'competition 40 N the ceiling is about 0.81 m/s and asking for '
                'more just parks the velocity loop in saturation.'
            ),
        ),
        Node(
            package='competitor_controller',
            executable='gamepad_controller',
            name='gamepad_controller',
            output='screen',
            # emulate_tty keeps the log readable when this is the thing on the
            # screen at a stand.
            emulate_tty=True,
            parameters=[{
                'use_sim_time': True,
                'device': LaunchConfiguration('device'),
                # Without the explicit value_type these arrive as strings and
                # the node's double parameters reject them.
                'heave_sign': ParameterValue(
                    LaunchConfiguration('heave_sign'), value_type=float),
                'cruise_speed': ParameterValue(
                    LaunchConfiguration('cruise_speed'), value_type=float),
            }],
        ),
    ])
