import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    
    rviz_config = os.path.join(get_package_share_directory('object_detection'),'rviz','object_det_real.rviz')

    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom',
        output='screen',
        emulate_tty=True,
        # arguments=[tx, ty, tz, yaw, pitch, roll, source_frame, target_frame]
        arguments=['0.5', '0.605', '0.1', '-3.142', '0.0', '-1.745', 'base_link', 'camera_depth_optical_frame']
    )

    object_detection_real_node = Node(
        package = 'object_detection',
        executable = 'object_detection_real',
        name = 'object_detection_real',
        output = 'screen',
        parameters = [{'use_sim_time': False}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        static_tf_pub,
        object_detection_real_node,
        rviz_node
    ])