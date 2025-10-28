# *********************************************************************************************************************
# Copyright [2025] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
#
# The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
# and/or its licensors ("Renesas") and subject to statutory and contractual protections.
#
# Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
# display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
# purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
# SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
# NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
# INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
# OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
# be subject to different terms.
# *********************************************************************************************************************

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource

def generate_launch_description():

    # Create image publisher node
    pkg_dir = get_package_share_directory('rzv_object_detection')
    test_image_path = os.path.join(pkg_dir, 'config/test/street.jpg')
    image_publisher_node = Node(
        package='image_publisher',
        executable='image_publisher_node',
        name='image_publisher',
        parameters=[{
            'filename': test_image_path,
            'publish_rate': 1.0
        }],
        remappings=[
            # publish camera info and image raw topics
            ('/camera_info', '/camera_info'),
             ('/image_raw', '/image_raw'),
       ]
    )

    # Create object detection node
    object_detection_node = Node(
        package='rzv_object_detection',
        executable='yolox_object_detection',
        name='object_detection',
        parameters=[{
            'model_type': 'yolox_pascal_voc',
            'processing_queue_size': 1,
            'confidence_threshold': 0.5,
            'iou_threshold': 0.3,
        }],
        remappings=[
            # subscribe to image raw topic
            ('/image_raw', '/image_raw'),
            # publish bounding box topic
            ('/bounding_box', '/object_detection/bounding_box'),
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

    # Foxglove bridge for visualization in Foxglove Studio
    foxglove_bridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
        )
    )

    # Create and return launch description
    return LaunchDescription([
        image_publisher_node,
        object_detection_node,
        foxglove_hand_bbox_publisher_node,
        foxglove_bridge_launch
    ])
