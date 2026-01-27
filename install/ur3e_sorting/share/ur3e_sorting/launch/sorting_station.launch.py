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
    world_file_arg = DeclareLaunchArgument(
        'world_file', default_value='pick_and_place_demo.world', description='World file to launch'
    )

    # 1. Launch Gazebo Simulation (ur_gazebo)
    # This brings up the robot, controllers, and MoveIt
    ur_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ur_gazebo_pkg, 'launch', 'ur.gazebo.launch.py'])
        ),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'world_file': LaunchConfiguration('world_file'),
            'launch_rviz': 'true',
            'spawn_x': '0.0',
            'spawn_y': '0.0',  # Revert to 0.0 (URDF handles offset)
            'spawn_z': '0.799',  # Table height approx 0.8m
            'spawn_yaw': '0.0',
        }.items()
    )

    # 2. Perception Node (USING YOLO)
    # Replaces the old perception_node
    perception_node = Node(
        package='ur3e_sorting',
        executable='yolo_detector', # Using the YOLO script (setup.py entry point)
        name='perception_node', # Naming it perception_node to be consistent
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 3. Sorting Node (Pick and Place Logic)
    sorting_node = Node(
        package='ur3e_sorting',
        executable='sorting_node',
        name='sorting_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        ur_type_arg,
        world_file_arg,
        ur_gazebo_launch,
        perception_node,
        sorting_node
    ])
