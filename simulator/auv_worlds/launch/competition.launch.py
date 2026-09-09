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
import xml.etree.ElementTree as ET

import yaml

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
    lifecycle = LaunchConfiguration('lifecycle').perform(context).lower() in ('true', '1', 'yes')

    seed = None if seed_arg in ('', 'auto') else int(seed_arg)

    try:
        run = generate_world(config_path, seed=seed, output_dir=output_dir)
    except LayoutError as exc:
        # Raising here aborts the launch with the YAML problem front and centre,
        # rather than handing Gazebo a world that was never written.
        raise RuntimeError(f'competition config is invalid: {exc}') from exc

    # The run manager talks to Gazebo's own services, which are namespaced by the
    # world's name rather than by the file it came from. Read it rather than
    # hardcode "underwater_pool", so changing base_world does not silently break
    # reset_run.
    world_name = ET.parse(run['world']).getroot().find('world').get('name')

    # The camera follows the vehicle by MODEL NAME, which the config owns. Read
    # it rather than hardcode "auv", for the same reason world_name is read
    # above: renaming the vehicle should not silently leave the camera pointing
    # at nothing.
    with open(config_path) as handle:
        vehicle_name = str(
            ((yaml.safe_load(handle) or {}).get('vehicle') or {}).get('name', 'auv'))

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

    torpedo_flagger = Node(
        package='auv_scoring',
        executable='torpedo_flagger',
        name='torpedo_flagger',
        parameters=[{
            'ground_truth_file': run['ground_truth'],
            'config_file': config_path,
            'use_sim_time': True,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('scoring')),
    )

    # Points the GUI camera at the vehicle and keeps it there, and serves the
    # FollowCam panel's buttons. A SEPARATE PROCESS on purpose, and it has to
    # be: /gui/follow is advertised by the GUI, and gz-transport will not route
    # a request from a node back to a service advertised by its own process, so
    # the panel cannot call it and something outside has to. See the script.
    #
    # Only with a GUI - there is no camera to point in headless mode.
    camera_director = ExecuteProcess(
        cmd=['python3',
             os.path.join(pkg_auv_worlds, 'scripts', 'camera_director.py'),
             '--target', vehicle_name,
             '--offset', '-2.0,1.0,1.0'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Task 5. Watches the four pickups' odometry rather than a contact sensor -
    # see the module docstring for why a placed object is a pose question and a
    # thrown marker is a contact one.
    octagon_flagger = Node(
        package='auv_scoring',
        executable='octagon_flagger',
        name='octagon_flagger',
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
    #
    # start_disarmed follows the lifecycle argument: with a run manager on the
    # graph the thrusters belong to /simulator/start_run, and without one nobody
    # would ever unkill them.
    simulator_bridge = Node(
        package='auv_simbridge',
        executable='simulator_bridge',
        name='simulator_bridge',
        parameters=[{'use_sim_time': True, 'start_disarmed': lifecycle}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('simulator_bridge')),
    )

    # Owns the run: start/end/reset, the clock, and the thruster arming. It reads
    # the same ground truth the flaggers do, because a reset re-samples the
    # course from the layout that produced it.
    run_manager = Node(
        package='auv_scoring',
        executable='run_manager',
        name='run_manager',
        parameters=[{
            'ground_truth_file': run['ground_truth'],
            'config_file': config_path,
            'world': world_name,
            'use_sim_time': True,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('lifecycle')),
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
        torpedo_flagger,
        octagon_flagger,
        camera_director,
        score_keeper,
        simulator_bridge,
        run_manager,
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
            'lifecycle',
            default_value='true',
            description='Run the start_run/end_run/reset_run manager, and hold the '
                        'thrusters dead until start_run. false restores the old '
                        'free-running behaviour.',
        ),
        DeclareLaunchArgument(
            'simulator_bridge',
            default_value='false',
            description='Also start auv_simbridge/simulator_bridge',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
