"""Launch an RGB-D camera. Currently configured for RealSense device."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    log_level = LaunchConfiguration('log_level', default='info')
    config_rs_camera = LaunchConfiguration('config_rs_camera', default=os.path.join(
        get_package_share_directory('drl_grasping'), 'config', 'rs_camera.yaml'))
    json_file_path = LaunchConfiguration('json_file_path', default=os.path.join(get_package_share_directory('drl_grasping'),
                                                                                'config', 'd435_rs_config.json'))

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description="If true, use simulated clock"),
        DeclareLaunchArgument(
            'log_level',
            default_value=log_level,
            description="Log level of all nodes launched by this script"),
        DeclareLaunchArgument(
            'config_rs_camera',
            default_value=config_rs_camera,
            description="Config for RealSense camera"),
        DeclareLaunchArgument(
            'json_file_path',
            default_value=json_file_path,
            description="JSON Config for RealSense camera"),

        # RealSense node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace="/rs_camera",
            output='screen',
            parameters=[config_rs_camera,
                        {'json_file_path': json_file_path},
                        ],
            remappings=[('depth/color/points', '/rgbd_camera/points')],
            emulate_tty=True,
        ),

        # Transformations (world <-> rs_camera)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_base_link_calib_board',
            namespace='',
            output='screen',
            parameters=[],
            arguments=['0.15200', '0.29738', '0.0',
                       '-0.78539816', '0.0', '0.0',
                       'base_link', 'calib_board'],
            remappings=[],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_calib_board_rs_camera',
            namespace='',
            output='screen',
            parameters=[],
            arguments=['0.36', '0.69', '0.56',
                       '-0.2783', '-0.1572', '0.8425', '-0.4335',
                       'calib_board', 'rs_camera'],
            remappings=[],
        ),
    ])
