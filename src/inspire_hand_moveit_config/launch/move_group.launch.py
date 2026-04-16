"""
move_group.launch.py — starts the MoveIt move_group node only.

Assumes the hardware stack (ros2_control_node, joint_state_broadcaster,
left_gripper_controller) is already running via:

    ros2 launch inspire_hand inspire_hand.launch.py

Launch arguments mirror inspire_hand.launch.py so the robot_description
parameters stay consistent.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml(package_name: str, rel_path: str) -> dict:
    abs_path = os.path.join(get_package_share_directory(package_name), rel_path)
    with open(abs_path) as f:
        return yaml.safe_load(f) or {}


def _load_file(package_name: str, rel_path: str) -> str:
    abs_path = os.path.join(get_package_share_directory(package_name), rel_path)
    with open(abs_path) as f:
        return f.read()


def generate_launch_description():
    pkg_hand = FindPackageShare("inspire_hand")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([pkg_hand, "urdf", "inspire_hand.ros2_control.xacro"]),
            " port:=",           LaunchConfiguration("port"),
            " baudrate:=",       LaunchConfiguration("baudrate"),
            " left_gripper_id:=",  LaunchConfiguration("left_gripper_id"),
            " right_gripper_id:=", LaunchConfiguration("right_gripper_id"),
        ]),
        value_type=str,
    )

    srdf          = _load_file("inspire_hand_moveit_config", "config/inspire_hand.srdf")
    kinematics    = _load_yaml("inspire_hand_moveit_config", "config/kinematics.yaml")
    joint_limits  = _load_yaml("inspire_hand_moveit_config", "config/joint_limits.yaml")
    ompl          = _load_yaml("inspire_hand_moveit_config", "config/ompl_planning.yaml")
    controllers   = _load_yaml("inspire_hand_moveit_config", "config/moveit_controllers.yaml")

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description":          robot_description},
            {"robot_description_semantic": srdf},
            {"robot_description_kinematics": kinematics},
            {"robot_description_planning":   joint_limits},
            # planning_pipelines is a flat list; default_planning_pipeline names the default.
            {"planning_pipelines": ["ompl"], "default_planning_pipeline": "ompl"},
            # ompl_planning.yaml has NO top-level "ompl:" key; wrap it here so params
            # land at ompl.planning_plugins, ompl.request_adapters, etc.
            {"ompl": ompl},
            # controllers yaml includes trajectory_execution settings + controller list
            controllers,
            {
                # Hardware controllers are managed by inspire_hand.launch.py.
                "moveit_manage_controllers": False,
                # Publish SRDF on /robot_description_semantic for RViz.
                "publish_robot_description_semantic": True,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("port",             default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate",         default_value="115200"),
        DeclareLaunchArgument("left_gripper_id",  default_value="1"),
        DeclareLaunchArgument("right_gripper_id", default_value="2"),
        move_group,
    ])
