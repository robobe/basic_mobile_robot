from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

PACKAGE = "basic_mobile_robot"
WORLD = "empty.world"


def generate_launch_description():

    ld = LaunchDescription()

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg = get_package_share_directory(PACKAGE)

    world = os.path.join(pkg, "worlds", WORLD),

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"),
        ),
        launch_arguments={'verbose': "true", 'world': world}.items()
    )

    
    ld.add_action(gazebo)
    
    return ld