#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    smart_pkg = get_package_share_directory('smart_community')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    # ==============================
    # 文件路径
    # ==============================
    competition_launch = os.path.join(
        smart_pkg,
        'launch',
        'competition.launch.py'
    )

    nav2_params = os.path.join(
        smart_pkg,
        'config',
        'nav2_params.yaml'
    )

    map_yaml = '/home/xjy/smart_community_map.yaml'

    # ==============================
    # 1. Gazebo + TurtleBot3
    # competition.launch.py 中已经包含：
    # Gazebo、机器人、initial_pose
    # ==============================
    competition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            competition_launch
        )
    )

    # ==============================
    # 2. Map Server
    # ==============================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': map_yaml,
                'use_sim_time': True
            }
        ]
    )

    # ==============================
    # 3. AMCL
    # ==============================
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'base_frame_id': 'base_footprint',
                'odom_frame_id': 'odom',
                'global_frame_id': 'map',
                'scan_topic': '/scan'
            }
        ]
    )

    # ==============================
    # 4. 自动激活 map_server
    # ==============================
    activate_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server']
            }
        ]
    )

    # ==============================
    # 5. 自动激活 AMCL
    # ==============================
    activate_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['amcl']
            }
        ]
    )

    # ==============================
    # 6. Nav2
    # 延迟启动，等待地图和 AMCL
    # ==============================
    navigation = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        nav2_pkg,
                        'launch',
                        'navigation_launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': 'True',
                    'params_file': nav2_params
                }.items()
            )
        ]
    )


    # ==============================
    # 9. RViz
    # ==============================
    rviz = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )

    return LaunchDescription([

        # 立即启动 Gazebo
        competition,

        # 2 秒后启动地图
        TimerAction(
            period=2.0,
            actions=[
                map_server,
                activate_map
            ]
        ),

        # 4 秒后启动 AMCL
        TimerAction(
            period=4.0,
            actions=[
                amcl,
                activate_amcl
            ]
        ),
        navigation,

        rviz
    ])
