import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

PACKAGE = "basic_mobile_robot"
WORLD = "empty.world"
SDF = 'robot.urdf.xacro'

def generate_launch_description():
    pkg = get_package_share_directory(PACKAGE)

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_ros2_control = LaunchConfiguration('use_ros2_control', default=False)

    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true')
    use_ros2_control_arg = DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Use ros2_control if true')

    # Process the URDF file
    xacro_file = os.path.join(pkg,'description', SDF)
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_ros2_control_arg)
    ld.add_action(node_robot_state_publisher)
    return ld