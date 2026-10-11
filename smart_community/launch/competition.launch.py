import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # -----------------------------
    # 获取 ROS2 package 路径
    # -----------------------------
    smart_community_pkg = get_package_share_directory(
        'smart_community'
    )

    gazebo_ros_pkg = get_package_share_directory(
        'gazebo_ros'
    )

    turtlebot3_gazebo_pkg = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    # -----------------------------
    # 比赛地图
    # -----------------------------
    world_file = os.path.join(
        smart_community_pkg,
        'worlds',
        'competition.world'
    )

    # -----------------------------
    # TurtleBot3 模型
    # -----------------------------
    robot_sdf = os.path.join(
        smart_community_pkg,
        'models',
        'turtlebot3_burger_camera',
        'model.sdf'
    )



    # -----------------------------
    # robot_state_publisher 发布TF
    # -----------------------------
    robot_description = Command([
        'xacro ',
        '/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf'
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': ParameterValue(
                    robot_description,
                    value_type=str
                ),
                'use_sim_time': True
            }
        ],
        output='screen'
    )


    # -----------------------------
    # 启动 Gazebo
    # -----------------------------
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

    # -----------------------------
    # 生成测试机器人
    # -----------------------------
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',

            '-file', robot_sdf,

            '-x', '1.76766',
            '-y', '-1.68207',
            '-z', '0.05',

            '-Y', '1.57296'
        ],
        output='screen'
    )

    # -----------------------------
    # 自动设置 AMCL 的 P0 初始位姿
    # 节点会等待 AMCL 启动后再发送
    # -----------------------------
    initial_pose = Node(
        package='smart_community',
        executable='initial_pose',
        output='screen'
    )

    return LaunchDescription([

        SetEnvironmentVariable(
            'TURTLEBOT3_MODEL',
            'burger'
        ),

        robot_state_publisher,

        gazebo,

        spawn_robot,

        initial_pose
    ])
