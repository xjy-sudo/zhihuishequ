import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    smart_pkg = get_package_share_directory('smart_community')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    world_file = os.path.join(
        smart_pkg,
        'worlds',
        'community.world'
    )

    robot_sdf = os.path.join(
        turtlebot3_gazebo_pkg,
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_pkg,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file
        }.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', robot_sdf,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'TURTLEBOT3_MODEL',
            'burger'
        ),
        gazebo,
        spawn_robot
    ])
