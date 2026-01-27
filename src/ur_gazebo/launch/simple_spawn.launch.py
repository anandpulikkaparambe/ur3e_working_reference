"""
Simple Gazebo launch file to test robot spawning.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Packages
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ur_description = get_package_share_directory('ur_description')
    pkg_robotiq_140 = get_package_share_directory('robotiq_2f_gripper_description')

    # Set Gazebo Resource Path
    gz_resource_path = (
        os.path.join(pkg_robotiq_140, '..') + ':' +
        os.path.join(pkg_ur_description, '..')
    )
    
    set_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gz_resource_path
    )

    # Robot Description (URDF)
    urdf_xacro_path = os.path.join(pkg_ur_description, 'urdf', 'ur_robotiq_140.urdf.xacro')
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', urdf_xacro_path, ' ur_type:=ur3 use_camera:=false'
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Gazebo
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf',
            'use_sim_time': 'true'
        }.items()
    )

    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'ur_robot',
            '-allow_renaming', 'true',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'
        ]
    )

    return LaunchDescription([
        set_env_vars,
        robot_state_publisher,
        start_gazebo,
        spawn_robot,
    ])
