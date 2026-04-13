from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port       = LaunchConfiguration("port")
    baud       = LaunchConfiguration("baudrate")
    left_id    = LaunchConfiguration("left_gripper_id")
    right_id   = LaunchConfiguration("right_gripper_id")
    pkg        = FindPackageShare("inspire_hand")

    urdf = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([pkg, "urdf", "inspire_hand.ros2_control.xacro"]),
            " port:=", port,
            " baudrate:=", baud,
            " left_gripper_id:=", left_id,
            " right_gripper_id:=", right_id,
        ]),
        value_type=str,
    )

    controllers_yaml = PathJoinSubstitution([pkg, "config", "gripper_controllers.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("port",             default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate",         default_value="115200"),
        DeclareLaunchArgument("left_gripper_id",  default_value="1"),
        DeclareLaunchArgument("right_gripper_id", default_value="2"),

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
            arguments=["left_gripper_controller"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["right_gripper_controller"],
            output="screen",
        ),
    ])
