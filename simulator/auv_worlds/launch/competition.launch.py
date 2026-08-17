"""Launch the competition simulator on a freshly randomised course.

A new world is generated on every launch, so the props never sit in exactly the
same place twice. Pass seed:=<n> to replay a specific course.

    ros2 launch auv_worlds competition.launch.py
    ros2 launch auv_worlds competition.launch.py seed:=1234
    ros2 launch auv_worlds competition.launch.py gui:=false
    ros2 launch auv_worlds competition.launch.py current:=false

The course layout and the water disturbance settings both live in
config/competition_config.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from auv_worlds.world_generator import generate_world, LayoutError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg_auv_worlds = get_package_share_directory('auv_worlds')

    config_path = LaunchConfiguration('config').perform(context)
    seed_arg = LaunchConfiguration('seed').perform(context).strip()
    output_dir = LaunchConfiguration('output_dir').perform(context).strip() or None
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1', 'yes')

    seed = None if seed_arg in ('', 'auto') else int(seed_arg)

    try:
        run = generate_world(config_path, seed=seed, output_dir=output_dir)
    except LayoutError as exc:
        # Raising here aborts the launch with the YAML problem front and centre,
        # rather than handing Gazebo a world that was never written.
        raise RuntimeError(f'competition config is invalid: {exc}') from exc

    # The Scoreboard GUI plugin lives in auv_gui; Gazebo only finds GUI plugins
    # on GZ_GUI_PLUGIN_PATH, which nothing sets for us.
    gui_plugin_dir = os.path.join(
        get_package_share_directory('auv_gui'), '..', '..', 'lib', 'auv_gui')
    gui_plugin_dir = os.path.normpath(gui_plugin_dir)
    existing_gui = os.environ.get('GZ_GUI_PLUGIN_PATH')
    os.environ['GZ_GUI_PLUGIN_PATH'] = (
        existing_gui + os.pathsep + gui_plugin_dir if existing_gui else gui_plugin_dir)

    # The generated world lives outside the share tree, so model:// URIs only
    # resolve if the models directory is on the resource path.
    models_dir = os.path.join(pkg_auv_worlds, 'models')
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH')
    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        existing + os.pathsep + models_dir if existing else models_dir)

    gz_args = [run['world'], ' -v 4']
    if not gui:
        gz_args.append(' -s')
    if LaunchConfiguration('paused').perform(context).lower() not in ('true', '1', 'yes'):
        # Start running. Without -r Gazebo comes up paused, sim time never
        # advances, and everything clocked on sim time - the water disturbance
        # included - sits silently waiting for a play button that a headless
        # competition run does not have.
        gz_args.append(' -r')

    gazebo = ExecuteProcess(
        cmd=[
            'nice', '-n', '-10',
            'ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
            ['gz_args:=', *gz_args],
            'on_exit_shutdown:=true',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='competition_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_auv_worlds, 'config',
                                        'bridge_topics_thrusters.yaml'),
            'use_sim_time': True,
        }],
        output='screen',
    )

    # Water disturbance. Seeded off the run seed so a replayed course sees the
    # same water, which scoring depends on. Reads its own `current:` block out
    # of the competition config.
    ocean_current = Node(
        package='auv_simbridge',
        executable='ocean_current',
        name='ocean_current',
        parameters=[{
            'config_file': config_path,
            'seed': run['seed'],
            'use_sim_time': True,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('current')),
    )

    # Task flaggers. These read the run's ground truth, so they live on the
    # scoring side of the fence, never in the competitor's stack.
    gate_flagger = Node(
        package='auv_scoring',
        executable='gate_flagger',
        name='gate_flagger',
        parameters=[{
            'ground_truth_file': run['ground_truth'],
            'config_file': config_path,
            'use_sim_time': True,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('scoring')),
    )

    slalom_flagger = Node(
        package='auv_scoring',
        executable='slalom_flagger',
        name='slalom_flagger',
        parameters=[{
            'ground_truth_file': run['ground_truth'],
            'config_file': config_path,
            'use_sim_time': True,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('scoring')),
    )

    teleop = Node(
        package='auv_simbridge',
        executable='teleop',
        name='auv_teleop',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('teleop')),
    )

    bin_flagger = Node(
        package='auv_scoring',
        executable='bin_flagger',
        name='bin_flagger',
        parameters=[{
            'ground_truth_file': run['ground_truth'],
            'config_file': config_path,
            'use_sim_time': True,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('scoring')),
    )

    score_keeper = Node(
        package='auv_scoring',
        executable='score_keeper',
        name='score_keeper',
        parameters=[{'config_file': config_path, 'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('scoring')),
    )

    # Off by default, matching the other launch files in this package, which
    # leave simulator_bridge to be started separately.
    simulator_bridge = Node(
        package='auv_simbridge',
        executable='simulator_bridge',
        name='simulator_bridge',
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('simulator_bridge')),
    )

    return [
        LogInfo(msg=f'[competition] seed {run["seed"]} -> {run["world"]}'),
        LogInfo(msg=f'[competition] ground truth: {run["ground_truth"]}'),
        gazebo,
        bridge,
        ocean_current,
        teleop,
        gate_flagger,
        slalom_flagger,
        bin_flagger,
        score_keeper,
        simulator_bridge,
    ]


def generate_launch_description():
    """Declare the launch arguments; the course is built in _launch_setup."""
    pkg_auv_worlds = get_package_share_directory('auv_worlds')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(pkg_auv_worlds, 'config', 'competition_config.yaml'),
            description='Competition config YAML: course layout and water disturbance',
        ),
        DeclareLaunchArgument(
            'seed',
            default_value='auto',
            description='Integer to replay a specific course; "auto" for a fresh one',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='',
            description='Where to write the generated world (default: $TMPDIR/matsya_competition)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='false runs Gazebo headless (server only)',
        ),
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            description='true starts Gazebo paused; sim time will not advance until you play',
        ),
        DeclareLaunchArgument(
            'current',
            default_value='true',
            description='false leaves the water still (the config has its own toggle too)',
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='true',
            description='Run the closed-loop keyboard teleop node',
        ),
        DeclareLaunchArgument(
            'scoring',
            default_value='true',
            description='Run the task flaggers (gate for now)',
        ),
        DeclareLaunchArgument(
            'simulator_bridge',
            default_value='false',
            description='Also start auv_simbridge/simulator_bridge',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
