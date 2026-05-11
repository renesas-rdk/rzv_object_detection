# *********************************************************************************************************************
#  Copyright (C) 2026 Renesas Electronics Corporation and/or its licensors.
#  SPDX-License-Identifier: AGPL-3.0-only
# *********************************************************************************************************************

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource

def generate_launch_description():
    # Create camera node using v4l2_camera
    # Puplishes images via default /image_raw topic
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'output_encoding': 'yuv422_yuy2',
            'image_size': [640, 480] # make sure to set the correct image size
        }]
    )

    # Create hand landmark node
    object_detection_node = Node(
        package='rzv_object_detection',
        executable='yolov8_object_detection',
        name='object_detection',
        parameters=[{
            'model_type': 'yolov8_rps',
            'processing_queue_size': 1,
            'confidence_threshold': 0.8,
            'iou_threshold': 0.3,
        }],
        remappings=[
            # subscribe to image raw topic
            ('/image_raw', '/image_raw'),
            # publish hand landmark topic
            ('/bounding_box', '/object_detection/bounding_box'),
            ('/object_detect', '/object_detection/rps_hand_detect'),
            # publish inference timing topic
            ('/inference_timing', '/object_detection/inference_timing'),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    # Add foxglove keypoint publisher node for bounding box visualization
    pkg_dir = get_package_share_directory('foxglove_keypoint_publisher')
    pose_config_path = os.path.join(pkg_dir, 'config/poses/bounding_box.yaml')
    foxglove_hand_bbox_publisher_node = Node(
        package='foxglove_keypoint_publisher',
        executable='foxglove_keypoint_publisher_node',
        name='foxglove_hand_bbox_publisher',
        parameters=[{
            'config_file': pose_config_path,
        }],
        remappings=[
            # subscribe to bounding box topic
            ('/keypoint_poses', '/object_detection/bounding_box'),
            # publish visualization topic
            ('/keypoint_visualization', '/bbox_visualization')
        ],
        output='screen'
    )

    # Add inference timing overlay node — displays pre/inference/post times as a Foxglove text overlay
    inference_timing_overlay_node = Node(
        package='foxglove_keypoint_publisher',
        executable='inference_timing_overlay_node',
        name='inference_timing_overlay_node',
        remappings=[
            # subscribe to inference timing topic
            ('/inference_timing', '/object_detection/inference_timing'),
            # publish overlay annotations
            ('/inference_timing_visualization', '/inference_timing_visualization'),
        ],
        output='screen'
    )

    # Foxglove bridge for visualization in Foxglove Studio
    foxglove_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
        )
    )

    # Create and return launch description
    return LaunchDescription([
        camera_node,
        object_detection_node,
        foxglove_hand_bbox_publisher_node,
        inference_timing_overlay_node,
        foxglove_bridge_launch
    ])
