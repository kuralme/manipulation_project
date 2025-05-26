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
    moveit_config_sim = MoveItConfigsBuilder("ur3e_sim", package_name="my_moveit_config").to_moveit_configs()
    
    use_real_robot = LaunchConfiguration('use_real_robot')
    declare_use_real_robot = DeclareLaunchArgument(
        'use_real_robot',
        default_value='False',
        description='Use real robot settings if True, simulation if False',
    )
    load_nodes_sim = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_real_robot])),
        actions=[
            Node(
                name="pick_and_place_perception_node",
                package="moveit2_scripts",
                executable="pick_and_place_perception_node",
                output="screen",
                parameters=[
                    moveit_config_sim.robot_description,
                    moveit_config_sim.robot_description_semantic,
                    moveit_config_sim.robot_description_kinematics,
                    {'use_sim_time': True},
                ],
            )
        ],
    )
    # The object detection launch file included
    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('object_detection'),
                'launch',
                'object_detection.launch.py'
            )
        )
    )
    # Declare the launch options
    ld = LaunchDescription()
    ld.add_action(object_detection_launch)
    ld.add_action(declare_use_real_robot)
    ld.add_action(load_nodes_sim)
    return ld