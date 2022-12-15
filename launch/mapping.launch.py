import os
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

PACKAGE = "basic_mobile_robot"
WORLD = "bug0.world"
MODEL = "basic_mobile_robot"
SDF = "model.sdf.xacro"


def generate_launch_description():
    pkg = get_package_share_directory(PACKAGE)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_config = os.path.join(pkg, "config", "slam_async.yaml")
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="use sim time"
    )

    slam = Node(
        parameters=[
          slam_config,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(slam)
    return ld
