import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Create camera node using v4l2_camera
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'output_encoding': 'yuv422_yuy2',
            'image_size': [640, 480] # check the camera resolution
        }]
    )

    # Create object detection node
    pkg_dir = get_package_share_directory('rzv_object_detection')
    model_path = os.path.join(pkg_dir, 'config/models/yolox_hand')
    object_detection_node = Node(
        package='rzv_object_detection',
        executable='object_detection',
        name='object_detection',
        parameters=[{
            'model_path': model_path,
            'processing_queue_size': 1,
            'confidence_threshold': 0.8,
            'iou_threshold': 0.45,
            'class_names': [
                'hand'
            ]
        }],
        remappings=[
            ('/image_raw', '/image_raw'),
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
            ('/keypoint_poses', '/object_detection/bounding_box'),
            ('/keypoint_visualization', '/keypoint_visualization')
        ],
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        camera_node,
        object_detection_node,
        foxglove_hand_bbox_publisher_node
    ])
