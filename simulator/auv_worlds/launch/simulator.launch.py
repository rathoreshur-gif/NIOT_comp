import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    # Get the package directory
    pkg_auv_worlds = get_package_share_directory('auv_worlds')
    
    # Path to your world file
    world_file = os.path.join(pkg_auv_worlds, 'worlds', 'test_matsya.sdf')
    
    # Path to models directory
    models_dir = os.path.join(pkg_auv_worlds, 'models')
    
    # Set Gazebo model path environment variable
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + models_dir
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_dir
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='/worlds/test_matsya.sdf '  
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Include Gazebo launch file
    gazebo = ExecuteProcess(
    cmd=[
        'nice', '-n', '-10',   # Lower = higher priority (-20 is highest)
        'ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
        ['gz_args:=', LaunchConfiguration('world'), ' -v 4'],
        'on_exit_shutdown:=true'
    ],
    output='screen'
)
    
    bridge_config = os.path.join(pkg_auv_worlds, 'config', 'bridge_topics_thrusters.yaml')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gazebo,
        bridge,
     
    ])
