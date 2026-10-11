from setuptools import find_packages, setup

package_name = 'smart_community'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            'share/' + package_name + '/launch',
            [
                'launch/community.launch.py',
                'launch/competition.launch.py',
                'launch/full_system.launch.py'
            ]
        ),

        (
            'share/' + package_name + '/config',
            [
                'config/nav2_params.yaml'
            ]
        ),

        (
            'share/' + package_name + '/models/turtlebot3_burger_camera',
            [
                'models/turtlebot3_burger_camera/model.sdf'
            ]
        ),

        (
            'share/' + package_name + '/worlds',
            [
                'worlds/community.world',
                'worlds/competition.world'
            ]
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='xjy',
    maintainer_email='xjy@todo.todo',

    description='Smart community competition simulation',

    license='TODO: License declaration',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'person_recognition = smart_community.person_recognition:main',
            'traffic_light_detection = smart_community.traffic_light_detection:main',
            'building_temperature_controller = smart_community.building_temperature_controller:main',
            'patrol_navigation = smart_community.patrol_navigation:main',
            'initial_pose = smart_community.initial_pose:main',
        ],
    },
)