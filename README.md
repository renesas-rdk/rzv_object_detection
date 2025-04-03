# RZV Object Detection

A ROS2 package for performing object detection on Renesas RZ/V2H platform. Implements various detection models and provides nodes for both static image and camera-based detection.

## Overview

This package provides ROS2 nodes for:
- General object detection using YOLOX Pascal VOC model
- Hand detection using YOLOX and Gold YOLOX models
- Real-time camera-based detection
- Static image-based detection

## Features

- Support for multiple detection models:
  - YOLOX Pascal VOC (20 classes)
  - YOLOX Hand Detection
  - Gold YOLOX Hand Detection
- Configurable detection parameters
- Integration with Foxglove Studio for visualization
- Multi-threaded processing support

## Launch Files

### Camera-based Detection
```bash
# Hand detection using camera input
ros2 launch rzv_object_detection camera_hand_detection.launch.py
```

### Static Image Detection
```bash
# Pascal VOC object detection on static image
ros2 launch rzv_object_detection static_object_detection.launch.py

# Hand detection on static image using YOLOX
ros2 launch rzv_object_detection static_hand_detection_yolox.launch.py

# Hand detection on static image using Gold YOLOX
ros2 launch rzv_object_detection static_hand_detection_gold_yolox.launch.py
```

## Configuration

Each launch file provides configurable parameters:
- Model type selection
- Processing queue size
- Confidence threshold
- IoU threshold
- Image input source
- Processing thread count

## Dependencies

- ROS2
- OpenCV
- rzv_model package
- Foxglove keypoint publisher (for visualization)
- v4l2_camera (for camera input)
- image_publisher (for static images)