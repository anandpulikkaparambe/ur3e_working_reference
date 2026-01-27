from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur_gazebo_pkg = FindPackageShare('ur_gazebo')

    # Launch Arguments
    ur_type_arg = DeclareLaunchArgument(
        'ur_type', default_value='ur3e', description='Robot type'
    )

    # 1. Launch Gazebo Simulation (ur_gazebo)
    # This brings up the robot, controllers, and MoveIt
    ur_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ur_gazebo_pkg, 'launch', 'ur.gazebo.launch.py'])
        ),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'launch_rviz': 'true',
            'world_file': 'pick_and_place_demo.world',
        }.items()
    )

    return LaunchDescription([
        ur_type_arg,
        ur_gazebo_launch
    ])
