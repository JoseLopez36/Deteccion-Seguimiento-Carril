from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'lane_following_sim'
    package_share = get_package_share_directory(package_name)

    world_path = os.path.join(package_share, 'worlds', 'lane_world.world')
    xacro_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        'urdf',
        'vehicle.urdf.xacro',
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_path}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_path]),
            'use_sim_time': True,
        }],
        output='screen',
    )

    spawn_vehicle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'lane_vehicle',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
        ],
        output='screen',
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_vehicle,
    ])
