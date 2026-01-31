"""
Launch Gazebo simulation with a UR robot.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

def generate_launch_description():
    # Package names
    package_name_gazebo = 'ur_gazebo'
    package_name_moveit = 'moveit_config'

    # Get package paths as strings
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share_gazebo = get_package_share_directory(package_name_gazebo)
    pkg_share_moveit = get_package_share_directory(package_name_moveit)
    pkg_share_ur_description = get_package_share_directory("ur_description")

    pkg_share_robotiq_140 = get_package_share_directory("robotiq_2f_gripper_description")
    

    # Paths
    gazebo_models_path = os.path.join(pkg_share_gazebo, 'models')
    gazebo_worlds_path = os.path.join(pkg_share_gazebo, 'worlds')
    ros_gz_bridge_config_file_path = os.path.join(pkg_share_gazebo, 'config', 'ros_gz_bridge.yaml')

    # Launch Configurations
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    ur_type = LaunchConfiguration('ur_type')
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument("robot_name", default_value="ur", description="The name for the robot"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"),
        DeclareLaunchArgument("world_file", default_value="pick_and_place_demo.world", description="World file name"),
        DeclareLaunchArgument("ur_type", default_value="ur3", description="Type/series of UR robot"),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument("spawn_x", default_value="0.0", description="Robot spawn X"),
        
    ]

    ld = LaunchDescription(declared_arguments)

    # Spawn Configs
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    # Set Gazebo Resource Path for Meshes
    # We append the Parent Directories of the packages so that "package://pkg_name" resolution works
    # Gazebo usually looks into paths in GZ_SIM_RESOURCE_PATH.
    gz_resource_path = (
        os.path.join(pkg_share_robotiq_140, '..') + ':' +
        os.path.join(pkg_share_ur_description, '..') + ':' +
        os.path.join(pkg_share_ur3e_sorting, '..') + ':' +
        gazebo_models_path
    )
    
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gz_resource_path
    )

    # Robot Description (URDF)
    urdf_xacro_path = os.path.join(pkg_share_ur_description, "urdf", "ur_robotiq_140.urdf.xacro")
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", urdf_xacro_path, " ur_type:=", ur_type, " use_camera:=false"
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Gazebo ROS Bridge
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config_file_path}],
        output='screen'
    )

    # Controller Manager
    # Controller Manager is handled by the Gazebo plugin (gz_ros2_control)
    # controller_manager_node = Node(...) 

    # Controllers
    controllers = ["joint_state_broadcaster", "arm_controller", "gripper_controller"]
    delays = [15.0, 20.0, 25.0]

    for controller, delay in zip(controllers, delays):
        ld.add_action(
            RegisterEventHandler(
                OnProcessStart(
                    target_action=start_gazebo_ros_bridge_cmd,
                    on_start=[
                        TimerAction(
                            period=delay,
                            actions=[
                                Node(
                                    package="controller_manager",
                                    executable="spawner",
                                    arguments=[controller, "-c", "/controller_manager"],
                                    parameters=[{'use_sim_time': True}],
                                    output='screen'
                                )
                            ]
                        )
                    ]
                )
            )
        )

    # Gazebo launch
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', PathJoinSubstitution([gazebo_worlds_path, world_file])],
            'use_sim_time': 'true'
        }.items()
    )

    # Spawn Robot
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
            '-R', '0.0', '-P', '0.0', '-Y', spawn_yaw
        ]
    )

    # MoveIt Config
    # Now that we have path strings, we can pass them to MoveItConfigsBuilder better?
    # Or just use the one that works.
    # MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name=package_name_moveit)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    
    # Overriding robot_description to ensure it uses the one with the gripper
    moveit_params = moveit_config.to_dict()
    moveit_params.update(robot_description) 

    run_move_group_node = Node(
        package="moveit_ros_move_group",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(pkg_share_moveit, "config", "moveit.rviz")],
        parameters=[
            moveit_params,
        ],
    )

    # Static TF Publisher (World -> Base Link)
    # Essential to tell MoveIt where the robot base is relative to world
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', spawn_x, '--y', spawn_y, '--z', '0.8', '--yaw', spawn_yaw, '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'base_link'],
        output='screen'
    )
    
    # Add actions
    ld.add_action(set_env_vars_resources)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(tf_node)
    # Controller Manager is internal to Gazebo plugin, no need to add external node.
    ld.add_action(run_move_group_node)
    ld.add_action(rviz_node)

    return ld
