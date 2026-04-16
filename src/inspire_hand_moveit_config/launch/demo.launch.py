"""
demo.launch.py — all-in-one launch: hardware + move_group + RViz.

Usage:
    ros2 launch inspire_hand_moveit_config demo.launch.py [port:=/dev/ttyUSB0]

This is a convenience wrapper. For production use, run the hardware and
move_group in separate terminals so you can restart move_group independently.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    pkg_hand   = FindPackageShare("inspire_hand")
    pkg_moveit = FindPackageShare("inspire_hand_moveit_config")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([pkg_hand, "urdf", "inspire_hand.ros2_control.xacro"]),
            " port:=",             LaunchConfiguration("port"),
            " baudrate:=",         LaunchConfiguration("baudrate"),
            " left_gripper_id:=",  LaunchConfiguration("left_gripper_id"),
            " right_gripper_id:=", LaunchConfiguration("right_gripper_id"),
        ]),
        value_type=str,
    )

    srdf         = _load_file("inspire_hand_moveit_config", "config/inspire_hand.srdf")
    kinematics   = _load_yaml("inspire_hand_moveit_config", "config/kinematics.yaml")
    joint_limits = _load_yaml("inspire_hand_moveit_config", "config/joint_limits.yaml")
    ompl         = _load_yaml("inspire_hand_moveit_config", "config/ompl_planning.yaml")
    controllers  = _load_yaml("inspire_hand_moveit_config", "config/moveit_controllers.yaml")

    # ── Hardware stack ──────────────────────────────────────────────────────
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_hand, "launch", "inspire_hand.launch.py"])
        ),
        launch_arguments={
            "port":             LaunchConfiguration("port"),
            "baudrate":         LaunchConfiguration("baudrate"),
            "left_gripper_id":  LaunchConfiguration("left_gripper_id"),
            "right_gripper_id": LaunchConfiguration("right_gripper_id"),
        }.items(),
    )

    # ── MoveIt move_group ───────────────────────────────────────────────────
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description":            robot_description},
            {"robot_description_semantic":   srdf},
            {"robot_description_kinematics": kinematics},
            {"robot_description_planning":   joint_limits},
            {"planning_pipelines": ["ompl"], "default_planning_pipeline": "ompl"},
            {"ompl": ompl},
            controllers,
            {
                "moveit_manage_controllers": False,
                "publish_robot_description_semantic": True,
            },
        ],
    )

    # ── RViz with MoveIt plugin ─────────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([pkg_moveit, "config", "moveit.rviz"])],
        parameters=[
            {"robot_description":            robot_description},
            {"robot_description_semantic":   srdf},
            {"robot_description_kinematics": kinematics},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("port",             default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate",         default_value="115200"),
        DeclareLaunchArgument("left_gripper_id",  default_value="1"),
        DeclareLaunchArgument("right_gripper_id", default_value="2"),
        hardware,
        move_group,
        rviz,
    ])
