import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, LogInfo

PACKAGE = "basic_mobile_robot"
SDF = "arg.sdf.xacro"


def generate_launch_description():
    with_grip = LaunchConfiguration('with_gripper')

    pkg = get_package_share_directory(PACKAGE)
    robot_description_path = os.path.join(pkg, "demos", "xacros", SDF)
    robot_description = Command(['xacro ', \
        robot_description_path, " ", "with_gripper:=", with_grip])

    log = LogInfo(msg=robot_description)
    arg = DeclareLaunchArgument(
            'with_gripper',
            default_value='true',
            description='Use grip')

    return LaunchDescription([
        arg,
        log
    ])