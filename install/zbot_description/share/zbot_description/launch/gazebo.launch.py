from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('zbot_description')
    rviz_config = os.path.join(pkg_path, 'rviz', 'zbot.rviz')
    world_file = os.path.join(pkg_path, 'worlds', 'Empty.world')
    urdf_xacro = os.path.join(pkg_path, 'urdf', 'zbot.urdf.xacro')

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'zbot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Robot State Publisher (needed for RViz)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_xacro]),
                value_type=str
            )
        }]
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        #gazebo,
        robot_state_publisher,
        #spawn_entity,
        joint_state_publisher,
        rviz
    ])
