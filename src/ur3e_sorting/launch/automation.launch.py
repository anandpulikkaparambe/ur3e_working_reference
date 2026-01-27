from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur_gazebo_pkg = FindPackageShare('ur_gazebo')
    ur3e_sorting_pkg = FindPackageShare('ur3e_sorting')

    # Launch Arguments
    ur_type_arg = DeclareLaunchArgument(
        'ur_type', default_value='ur3e', description='Robot type'
    )
    
    # Use our conveyor world by default, as requested "without interlink" 
    # but needing the environment.
    world_file_arg = DeclareLaunchArgument(
        'world_file', default_value='conveyor.world', description='World file'
    )

    # 1. Launch Gazebo Simulation (ur_gazebo)
    ur_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ur_gazebo_pkg, 'launch', 'ur.gazebo.launch.py'])
        ),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'launch_rviz': 'true',
            'world_file': LaunchConfiguration('world_file'), # Pass conveyor.world
            'spawn_x': '0.0',
            'spawn_y': '0.3', 
            'spawn_z': '0.805', # On table (0.8) + slight buffer
            'spawn_yaw': '0.0'
        }.items()
    )

    # 2. Automation Node (Hardcoded Pick and Place)
    automation_node = Node(
        package='ur3e_sorting',
        executable='automation_node',
        name='automation_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        ur_type_arg,
        world_file_arg,
        ur_gazebo_launch,
        automation_node
    ])
