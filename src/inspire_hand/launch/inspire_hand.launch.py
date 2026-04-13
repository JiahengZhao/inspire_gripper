from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baudrate")
    gid  = LaunchConfiguration("gripper_id")
    pkg  = FindPackageShare("inspire_hand")

    urdf = Command([
        "xacro ",
        PathJoinSubstitution([pkg, "urdf", "inspire_hand.ros2_control.xacro"]),
        " port:=", port,
        " baudrate:=", baud,
        " gripper_id:=", gid,
    ])

    controllers_yaml = PathJoinSubstitution([pkg, "config", "gripper_controllers.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("port",       default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate",   default_value="115200"),
        DeclareLaunchArgument("gripper_id", default_value="1"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": urdf}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": urdf}, controllers_yaml],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_controller"],
            output="screen",
        ),
    ])
