from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

import os


def generate_launch_description():

    params_file = "/home/vmx/nav-diff-robot/nav2_params/nav2_params.yaml"
    map_file = "/home/vmx/nav-diff-robot/src/robot_core/maps/empty.yaml"

    description_pkg = get_package_share_directory("robot_description")
    control_pkg = get_package_share_directory("robot_control")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")

    sim_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true"
    )

    robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                control_pkg,
                "launch",
                "control.launch.py"
            )
        ),
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                description_pkg,
                "launch",
                "robot_description.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )

    map_tf = ExecuteProcess(
        cmd=[
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "0", "0", "0", "0", "0", "0",
            "map", "odom"
        ],
        output="screen"
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": map_file,
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool)
        }]
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "autostart": True,
            "node_names": ["map_server"]
        }]
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                "launch",
                "navigation_launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file
        }.items()
    )

    return LaunchDescription([
        robot_control,
        sim_arg,
        robot_description,
        map_tf,
        map_server,
        lifecycle_manager,
        navigation,
    ])