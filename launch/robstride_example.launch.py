from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("robstride_ros2"))
    socketcan_share = Path(get_package_share_directory("ros2_socketcan"))
    interface = LaunchConfiguration("interface")
    controllers = str(package_share / "config/controllers.yaml")
    description = Command(
        ["xacro ", str(package_share / "description/robstride_example.urdf.xacro")]
    )

    socketcan = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(str(socketcan_share / "launch/socket_can_bridge.launch.xml")),
        launch_arguments={"interface": interface}.items(),
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": description}],
        output="screen",
    )
    joint_states = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    position = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robstride_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("interface", default_value="can0"),
            socketcan,
            state_publisher,
            control_node,
            joint_states,
            RegisterEventHandler(OnProcessExit(target_action=joint_states, on_exit=[position])),
        ]
    )
