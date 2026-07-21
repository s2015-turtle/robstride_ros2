from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    example_launch = Path(get_package_share_directory("robstride_examples")) / (
        "launch/robstride_example.launch.py"
    )
    interface = LaunchConfiguration("interface")
    return LaunchDescription(
        [
            DeclareLaunchArgument("interface", default_value="can0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(example_launch)),
                launch_arguments={"interface": interface}.items(),
            ),
        ]
    )
