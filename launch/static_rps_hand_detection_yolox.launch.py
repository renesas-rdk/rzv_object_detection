# *********************************************************************************************************************
#  Copyright (C) 2026 Renesas Electronics Corporation and/or its licensors.
#  SPDX-License-Identifier: AGPL-3.0-only
# *********************************************************************************************************************

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='yolox_s_rps',
        description='Name of the soft objects detection model folder'
    )

    # Create image publisher node for static test image
    pkg_dir = get_package_share_directory('rzv_object_detection')
    test_image_path = os.path.join(pkg_dir, 'config/test/rps_game.jpg')

    image_publisher_node = Node(
        package='image_publisher',
        executable='image_publisher_node',
        name='image_publisher',
        parameters=[{
            'filename': test_image_path,
            'publish_rate': 1.0
        }],
        remappings=[
            ('/camera_info', '/camera_info'),
            ('/image_raw', '/image_raw'),
        ],
        output='screen'
    )

    # Create soft objects detection node
    yolox_rps_detection_node = Node(
        package='rzv_object_detection',
        executable='yolox_rps_detection',
        name='yolox_rps_detection',
        parameters=[{
            'model_type': LaunchConfiguration('model_type'),
            'processing_queue_size': 1,
            'confidence_threshold': 0.7,
            'iou_threshold': 0.45,
        }],
        remappings=[
            ('/image_raw', '/image_raw'),
            ('/bounding_box', '/yolox_rps_detection/bounding_box'),
            ('/inference_timing', '/yolox_rps_detection/inference_timing')
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    # Add foxglove keypoint publisher node for bounding box visualization
    foxglove_pkg_dir = get_package_share_directory('foxglove_keypoint_publisher')
    pose_config_path = os.path.join(foxglove_pkg_dir, 'config/poses/bounding_box.yaml')

    # Add inference timing overlay node — displays pre/inference/post times as a Foxglove text overlay
    inference_timing_overlay_node = Node(
        package='foxglove_keypoint_publisher',
        executable='inference_timing_overlay_node',
        name='inference_timing_overlay_node',
        remappings=[
            # subscribe to inference timing topic
            ('/inference_timing',
             '/yolox_rps_detection/inference_timing'),
            # publish overlay annotations
            ('/inference_timing_visualization', '/inference_timing_visualization'),
        ],
        output='screen'
    )

    foxglove_bbox_publisher_node = Node(
        package='foxglove_keypoint_publisher',
        executable='foxglove_keypoint_publisher_node',
        name='foxglove_bbox_publisher',
        parameters=[{
            'config_file': pose_config_path,
        }],
        remappings=[
            ('/keypoint_poses', '/yolox_rps_detection/bounding_box'),
            ('/keypoint_visualization', '/bbox_visualization')
        ],
        output='screen'
    )

    # Foxglove bridge for visualization in Foxglove Studio
    foxglove_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            )
        )
    )

    return LaunchDescription([
        model_type_arg,
        image_publisher_node,
        yolox_rps_detection_node,
        foxglove_bbox_publisher_node,
        inference_timing_overlay_node,
        foxglove_bridge_launch
    ])
