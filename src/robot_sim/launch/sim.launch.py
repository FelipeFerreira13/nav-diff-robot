from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

import os
import yaml


def generate_launch_description():

    pkg_path = get_package_share_directory('robot_sim')
    description_pkg = get_package_share_directory('robot_description')
    core_pkg = get_package_share_directory('robot_core')
    control_pkg = get_package_share_directory('robot_control')
    localization_pkg = get_package_share_directory('robot_localization')




    world = os.path.join(pkg_path, 'world', 'world.world')
    # world = os.path.join(pkg_path, 'world', 'empty.world')

    use_sim_time = LaunchConfiguration('use_sim_time')

    sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world,
            '-s', 'libgazebo_ros_init.so',      # initializes Gazebo’s ROS interface, especially simulated time and ROS node integration
            '-s', 'libgazebo_ros_factory.so'    # lets ROS spawn models into Gazebo
        ],
        output='screen'
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                description_pkg,
                'launch',
                'robot_description.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    avoidance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                core_pkg,
                'launch',
                'avoidance.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )


    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                localization_pkg,
                'launch',
                'localization.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0.35',
            '-y', '0.35',
            '-z', '0.0',
            '-Y', '1.57'
        ],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )


    return LaunchDescription([
        sim_arg,
        gazebo,
        robot_description,
        avoidance,
        localization,
        spawn_entity,
        
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster],
            )
        ),

    ])