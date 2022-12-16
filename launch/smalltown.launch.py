import os

from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import xacro

PACKAGE = "basic_mobile_robot"
WORLD = "smalltown.world"
MODEL = "basic_mobile_robot"
SDF = "model.sdf.xacro"


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_nav2 = get_package_share_directory("nav2_bringup")
    pkg_nav2_bt = get_package_share_directory("nav2_bt_navigator")
    behavior_tree_xml_path = os.path.join(pkg_nav2_bt, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    pkg = get_package_share_directory(PACKAGE)
    rviz_config = os.path.join(pkg, "config", "rviz.rviz")
    ekf_file_path= os.path.join(pkg, "config", "ekf.yaml")
    map_yaml_file = os.path.join(pkg, "maps", "smalltown_world.yaml")
    nav2_params_path = os.path.join(pkg, 'config', 'nav2_params.yaml')
    resources = [os.path.join(pkg, "worlds")]

    resource_env = AppendEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH", value=":".join(resources)
    )

    models = [os.path.join(pkg, "models")]

    models_env = AppendEnvironmentVariable(
        name="GAZEBO_MODEL_PATH", value=":".join(models)
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py"),
        ),
        launch_arguments={"verbose": "true", "world": WORLD}.items(),
    )

    robot_description_path = os.path.join(pkg, "models", MODEL, SDF)
    doc = xacro.parse(open(robot_description_path))
    xacro.process_doc(doc)

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="use sim time"
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'use_sim_time': use_sim_time, 
                'robot_description': doc.toxml()
            }
        ],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
                ]
    )

    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_file_path, 
            {'use_sim_time': use_sim_time}
        ])

    # spawn_entity = Node(
    #     package="gazebo_ros",
    #     executable="spawn_entity.py",
    #     arguments=["-entity", "demo", "-topic", "robot_description"],
    #     output="screen",
    # )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    start_ros2_navigation_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_nav2, "launch", 'bringup_launch.py')),
    launch_arguments = {'namespace': "",
        'use_namespace': "False",
        'slam': "False",
        'map': "/home/user/nav2_ws/src/basic_mobile_robot/maps/smalltown_world.yaml",
        'use_sim_time': use_sim_time,
        'params_file': nav2_params_path,
        'default_bt_xml_filename': behavior_tree_xml_path,
        'autostart': "true"}.items())

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(resource_env)
    ld.add_action(models_env)
    ld.add_action(gazebo)
    ld.add_action(robot_localization)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz)
    ld.add_action(start_ros2_navigation_cmd)
    
    return ld
