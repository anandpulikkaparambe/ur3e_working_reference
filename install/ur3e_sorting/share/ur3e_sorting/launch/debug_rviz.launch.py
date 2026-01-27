from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
import yaml

def generate_launch_description():
    pkg_share_moveit = FindPackageShare('moveit_config').find('moveit_config')
    pkg_share_ur_description = FindPackageShare("ur_description").find("ur_description")

    # Robot Description
    urdf_xacro_path = os.path.join(pkg_share_ur_description, "urdf", "ur.urdf.xacro")
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", urdf_xacro_path, " ur_type:=ur3e"
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # SRDF
    srdf_path = os.path.join(pkg_share_moveit, "config", "ur.srdf")
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ", srdf_path
    ])
    robot_description_semantic = {'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)}

    # Kinematics
    kinematics_yaml_path = os.path.join(pkg_share_moveit, "config", "kinematics.yaml")
    with open(kinematics_yaml_path, 'r') as file:
        robot_description_kinematics = {"robot_description_kinematics": yaml.safe_load(file)}
    
    # RViz
    rviz_config_file = PathJoinSubstitution([pkg_share_moveit, "config", "moveit.rviz"])

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
            ],
        )
    ])
