# RZV Object Detection

A ROS2 package for performing object detection on Renesas RZ/V2H platform. Implements various detection models and provides nodes for both static image and camera-based detection.

## Overview

This package provides ROS2 nodes for:
- General object detection using YOLOX Pascal VOC model
- Hand detection using YOLOX and Gold YOLOX models
- Rock-Paper-Scissors hand detection using YOLOv8 models
- Real-time camera-based detection
- Static image-based detection

## Features

- Support for multiple detection models:
  - YOLOX Pascal VOC (20 classes)
  - YOLOX Hand Detection
  - Gold YOLO Hand Detection
  - YOLOv8 RPS Hand Detection
- Configurable detection parameters
- Integration with Foxglove Studio for visualization
- Multi-threaded processing support

## Nodes

### ObjectDetection

This node performs object detection on incoming image streams and publishes bounding box information.

#### Parameters

| Parameter Name | Data Type | Default | Description |
|----------------|-----------|---------|-------------|
| `model_path` | string | "" | Path to the model file |
| `model_type` | string | "yolox_pascal_voc" | Type of model to use (yolox_pascal_voc, yolox_hand, gold_yolox_hand, yolov8_rps) |
| `processing_queue_size` | int | 5 | Size of the image processing queue |
| `confidence_threshold` | float | 0.5 | Threshold for detection confidence |
| `iou_threshold` | float | 0.45 | Threshold for IoU in NMS |
| `class_names` | string[] | [] | Optional custom class names |
| `processing_threads` | int | 1 | Number of processing threads |

#### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/image_raw` | `sensor_msgs/msg/Image` | Input images for object detection |

#### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `bounding_box` | `geometry_msgs/msg/PoseArray` | Detection results as pose arrays |
| `object_detect` | `std_msgs/msg/String` | Detected object name |

## Launch Files

### Camera-based Detection
```bash
# Hand detection using camera input
ros2 launch rzv_object_detection camera_hand_detection_yolox.launch.py

# RPS hand detection using camera input
ros2 launch rzv_object_detection camera_rps_hand_detection_yolov8.launch.py
```

### Static Image Detection
```bash
# Pascal VOC object detection on static image using YOLOX
ros2 launch rzv_object_detection static_object_detection_yolox.launch.py

# Hand detection on static image using YOLOX
ros2 launch rzv_object_detection static_hand_detection_yolox.launch.py

# Hand detection on static image using Gold YOLO
ros2 launch rzv_object_detection static_hand_detection_gold_yolo.launch.py

# RPS hand detection on static image using YOLOv8
ros2 launch rzv_object_detection static_rps_hand_detection_yolov8.launch.py
```

## Launch File Parameters

### camera_hand_detection_yolox.launch.py

| Node | Parameter | Default | Description |
|------|-----------|---------|-------------|
| v4l2_camera | video_device | /dev/video0 | Camera device path |
| v4l2_camera | output_encoding | yuv422_yuy2 | Image encoding format |
| v4l2_camera | image_size | [640, 480] | Camera resolution |
| yolox_object_detection | model_type | yolox_hand | Detection model type |
| yolox_object_detection | processing_queue_size | 1 | Queue size for processing |
| yolox_object_detection | confidence_threshold | 0.8 | Confidence threshold |
| yolox_object_detection | iou_threshold | 0.3 | IoU threshold for NMS |

### camera_rps_hand_detection_yolov8.launch.py

| Node | Parameter | Default | Description |
|------|-----------|---------|-------------|
| v4l2_camera | video_device | /dev/video0 | Camera device path |
| v4l2_camera | output_encoding | yuv422_yuy2 | Image encoding format |
| v4l2_camera | image_size | [640, 480] | Camera resolution |
| yolov8_object_detection | model_type | yolov8_rps | Detection model type |
| yolov8_object_detection | processing_queue_size | 1 | Queue size for processing |
| yolov8_object_detection | confidence_threshold | 0.8 | Confidence threshold |
| yolov8_object_detection | iou_threshold | 0.3 | IoU threshold for NMS |

### static_object_detection_yolox.launch.py

| Node | Parameter | Default | Description |
|------|-----------|---------|-------------|
| image_publisher | filename | config/test/street.jpg | Test image path |
| image_publisher | publish_rate | 1.0 | Image publishing rate in Hz |
| yolox_object_detection | model_type | yolox_pascal_voc | Detection model type |
| yolox_object_detection | processing_queue_size | 1 | Queue size for processing |
| yolox_object_detection | confidence_threshold | 0.5 | Confidence threshold |
| yolox_object_detection | iou_threshold | 0.3 | IoU threshold for NMS |

### static_hand_detection_yolox.launch.py

| Node | Parameter | Default | Description |
|------|-----------|---------|-------------|
| image_publisher | filename | config/test/hand_5.png | Test image path |
| image_publisher | publish_rate | 1.0 | Image publishing rate in Hz |
| yolox_object_detection | model_type | yolox_hand | Detection model type |
| yolox_object_detection | processing_queue_size | 1 | Queue size for processing |
| yolox_object_detection | confidence_threshold | 0.8 | Confidence threshold |
| yolox_object_detection | iou_threshold | 0.3 | IoU threshold for NMS |

### static_hand_detection_gold_yolo.launch.py

| Node | Parameter | Default | Description |
|------|-----------|---------|-------------|
| image_publisher | filename | config/test/hand_5.png | Test image path |
| image_publisher | publish_rate | 1.0 | Image publishing rate in Hz |
| yolox_object_detection | model_type | gold_yolo_hand | Detection model type |
| yolox_object_detection | processing_queue_size | 1 | Queue size for processing |
| yolox_object_detection | confidence_threshold | 0.4 | Confidence threshold |
| yolox_object_detection | iou_threshold | 0.45 | IoU threshold for NMS |

### static_rps_hand_detection_yolov8.launch.py

| Node | Parameter | Default | Description |
|------|-----------|---------|-------------|
| image_publisher | filename | config/test/hand_5.png | Test image path |
| image_publisher | publish_rate | 1.0 | Image publishing rate in Hz |
| yolov8_object_detection | model_type | yolov8_rps | Detection model type |
| yolov8_object_detection | processing_queue_size | 1 | Queue size for processing |
| yolov8_object_detection | confidence_threshold | 0.8 | Confidence threshold |
| yolov8_object_detection | iou_threshold | 0.3 | IoU threshold for NMS |

## Usage Examples

### Running with Custom Parameters

```bash
# Run with custom model type and thresholds
ros2 run rzv_object_detection yolox_object_detection --ros-args -p model_type:=yolox_hand -p confidence_threshold:=0.6 -p iou_threshold:=0.4

# Run with specific model path
ros2 run rzv_object_detection yolox_object_detection --ros-args -p model_path:=/path/to/custom/drpai/model
```

### Topic Remapping

```bash
# Remap input image topic
ros2 run rzv_object_detection yolox_object_detection --ros-args --remap /image_raw:=/camera/image_raw

# Remap output bounding box topic
ros2 run rzv_object_detection yolox_object_detection --ros-args --remap bounding_box:=/detections
```

### Launch File Customization

```bash
# Launch with custom camera device
ros2 launch rzv_object_detection camera_hand_detection_yolox.launch.py video_device:=/dev/video1

# Launch with custom image
ros2 launch rzv_object_detection static_object_detection_yolox.launch.py filename:=/path/to/custom/image.jpg

# Launch with modified thresholds
ros2 launch rzv_object_detection static_hand_detection_yolox.launch.py confidence_threshold:=0.7 iou_threshold:=0.5
```

## Integration with Other Packages

### Visualization with Foxglove

The package is designed to work with the foxglove_keypoint_publisher for visualization:

```bash
# First run object detection
ros2 launch rzv_object_detection static_object_detection_yolox.launch.py

# Then run the visualization node
ros2 run foxglove_keypoint_publisher foxglove_keypoint_publisher_node --ros-args -p config_file:=$(ros2 pkg prefix foxglove_keypoint_publisher)/share/foxglove_keypoint_publisher/config/poses/bounding_box.yaml --remap /keypoint_poses:=/object_detection/bounding_box
```

## Dependencies

- ROS2
- OpenCV
- rzv_model_utils_ros2 packages
- rzv_model and rzv_<AI-model-related> packages
- image_publisher (for static images)
- v4l2_camera (for camera input)
- foxglove_keypoint_publisher (for visualization)