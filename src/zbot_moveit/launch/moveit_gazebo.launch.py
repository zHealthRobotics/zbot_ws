import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="zbot",
            package_name="zbot_moveit"
        )
        .to_moveit_configs()
    )
    
    ompl_yaml = os.path.join(
        get_package_share_directory("zbot_moveit"),
        "config",
        "ompl_planning.yaml"
    )
    

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("zbot_control")
            + "/launch/gazebo.launch.py"
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            ompl_yaml,
            {"use_sim_time": True},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            get_package_share_directory("zbot_moveit")
            + "/config/moveit.rviz"
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {"use_sim_time": True},
        ],
    )

    commander_node = Node(
        package="my_robot_commander_cpp",
        executable="commander",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )
    

    return LaunchDescription([
        gazebo_launch,
        move_group_node,
        rviz_node,
        commander_node,  
    ])

