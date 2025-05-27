import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    moveit_config_real = MoveItConfigsBuilder("ur3e_real", package_name="real_moveit_config").to_moveit_configs()

    use_real_robot = LaunchConfiguration('use_real_robot')
    declare_use_real_robot = DeclareLaunchArgument(
        'use_real_robot',
        default_value='True',
        description='Use real robot settings if True, simulation if False',
    )
    load_nodes_real = GroupAction(
        condition=IfCondition(use_real_robot),
        actions=[
            Node(
                name="pick_and_place_perception_real_node",
                package="moveit2_scripts",
                executable="pick_and_place_perception_real_node",
                output="screen",
                parameters=[
                    moveit_config_real.robot_description,
                    moveit_config_real.robot_description_semantic,
                    moveit_config_real.robot_description_kinematics,
                    {'use_sim_time': False},
                ],
            )
        ],
    )
    # The object detection node to be launched first
    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('object_detection'),
                'launch',
                'object_detection_real.launch.py'
            )
        )
    )
    # Declare the launch options
    ld = LaunchDescription()
    ld.add_action(object_detection_launch)
    ld.add_action(declare_use_real_robot)
    ld.add_action(load_nodes_real)
    return ld