import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_share = FindPackageShare('zbot_control')
    pkg_description = FindPackageShare('zbot_description')
    
    rviz_config = PathJoinSubstitution([
        pkg_description,
        'rviz',
        'zbot.rviz'
    ])    
    
    urdf_xacro = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'zbot_control.urdf.xacro'
    ])


    world_file = PathJoinSubstitution([
        pkg_description, 'worlds', 'Empty.world'
    ])

    # Gazebo with saved world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': world_file
        }.items()
    )

    # Robot State Publisher
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



    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'zbot',

            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ]
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Load controllers AFTER spawn
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'joint_state_broadcaster'
        ],
        output='screen'
    )

    left_arm_trajectory_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'left_arm_trajectory_controller'
        ],
        output='screen'
    )

    left_gripper_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'left_gripper_controller'
        ],
        output='screen'
    )

    right_gripper_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'right_gripper_controller'
        ],
        output='screen'
    )

    right_arm_trajectory_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'right_arm_trajectory_controller'
        ],
        output='screen'
    )    

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        rviz,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    load_joint_state_broadcaster,
                    left_arm_trajectory_controller,
                    left_gripper_controller,
                    right_gripper_controller,
                    right_arm_trajectory_controller
                ]
            )
        )
    ])

