from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'depth_module.depth_profile': '480,270,30',
            'rgb_camera.color_profile': '480,270,30',
        }.items()
    )

    img_processor = Node(
        package='robot_arm_pkg',
        executable='img_processor_node',
        name='img_processor_node',
        output='screen'
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '0.0716',
            '0.0075',
            '0.0877',
            '0.0',      # yaw
            '0.0873',   # pitch
            '0.0',      # roll
            'base_frame',
            'camera_link'
        ],
        output='screen'
    )

    return LaunchDescription([
        realsense_camera,
        img_processor,
        camera_tf
    ])

