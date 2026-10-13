from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_vacuum_gripper",
            default_value="true",
            description="Vakum tutucuyu (vacuum gripper) aktif et",
        ),
    ]

    use_vacuum_gripper = LaunchConfiguration("use_vacuum_gripper")

    description_package = FindPackageShare("my_robot_cell_control")
    description_file = PathJoinSubstitution(
        [description_package, "urdf", "whole_cell_sim.urdf.xacro"]
    )
    rvizconfig_file = PathJoinSubstitution([description_package, "rviz", "urdf.rviz"])

    set_env = SetEnvironmentVariable("ENV_USE_VACUUM_GRIPPER", use_vacuum_gripper)

    robot_description = ParameterValue(
        Command([
            "xacro ", description_file, " ",
            "ur_type:=", "ur10e", " ",
            "use_vacuum_gripper:=", use_vacuum_gripper
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig_file],
    )

    return LaunchDescription(
        declared_arguments + [set_env, joint_state_publisher_gui_node, robot_state_publisher_node, rviz_node]
    )
