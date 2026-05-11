// *********************************************************************************************************************
//  Copyright (C) 2026 Renesas Electronics Corporation and/or its licensors.
//  SPDX-License-Identifier: AGPL-3.0-only
// *********************************************************************************************************************
#include "rzv_object_detection/yolox_object_detection_node.hpp"

#include <unistd.h>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <thread>
#include <vector>

#include "rzv_model/utils.hpp"
#include "rzv_model_utils_ros2/model_utils.hpp"

namespace rzv_object_detection
{

YoloXObjectDetection::YoloXObjectDetection() : Node("YoloXObjectDetection")
{
  RCLCPP_INFO(this->get_logger(), "Node object detection started!");

  // Declare parameters with default values
  this->declare_parameter("model_path", "");
  this->declare_parameter("model_type", "yolox_pascal_voc");
  this->declare_parameter("class_names", std::vector<std::string>{});
  this->declare_parameter("confidence_threshold", 0.5f);
  this->declare_parameter("iou_threshold", 0.45f);
  this->declare_parameter("processing_queue_size", 5);
  this->declare_parameter("processing_threads", 1);  // Default to 1 for sequential processing

  RCLCPP_INFO(this->get_logger(), " Get parameters");
  // Get parameters
  std::string model_path_param = this->get_parameter("model_path").as_string();
  model_type_ = this->get_parameter("model_type").as_string();
  auto class_names_param = this->get_parameter("class_names").as_string_array();
  confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
  iou_threshold_ = this->get_parameter("iou_threshold").as_double();
  int queue_size = this->get_parameter("processing_queue_size").as_int();

  // Load model config from YAML config
  // Fallback logic: User → YAML → default value
  auto object_model = rzv_model::UtilsROS::load_model_info(
    "rzv_object_detection", model_type_, model_path_param, class_names_param);  // Object model
  model_path_ = object_model.model_path;
  class_names_ = object_model.class_names;

  // Create a mutually exclusive callback group - ensures one callback runs at a time
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Configure QoS with queue depth and frame dropping policy
  auto qos_reliable_stream = rclcpp::QoS(queue_size);
  qos_reliable_stream.keep_last(queue_size);  // Only keep latest frames
  qos_reliable_stream.reliable();             // Or use best_effort() for more aggressive dropping
  qos_reliable_stream.durability_volatile();  // Don't persist old messages

  auto qos_sensor_data = rclcpp::QoS(rclcpp::KeepLast(1));
  qos_sensor_data.best_effort();          // Only keep latest message
  qos_sensor_data.durability_volatile();  // Or use best_effort() for more aggressive dropping

  // Set subscription options with callback group
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  // Create subscription
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", qos_reliable_stream,
    std::bind(&YoloXObjectDetection::process_image, this, std::placeholders::_1), options);

  // Create publisher
  bbox_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("bounding_box", qos_reliable_stream);
  diagnostic_timing_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "inference_timing", qos_sensor_data);

  // Log configuration
  RCLCPP_INFO(this->get_logger(), "ObjectDetection initialized");
  RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Model type: %s", model_type_.c_str());

  RCLCPP_INFO(this->get_logger(), "Image processing queue size: %d", queue_size);
  RCLCPP_INFO(this->get_logger(), "Confidence threshold: %.2f", confidence_threshold_);
  RCLCPP_INFO(this->get_logger(), "IoU threshold: %.2f", iou_threshold_);
  RCLCPP_INFO(this->get_logger(), "Number of classes: %zu", class_names_.size());
  RCLCPP_INFO(
    this->get_logger(), "Subscribing to image topic: %s", image_subscription_->get_topic_name());
  RCLCPP_INFO(
    this->get_logger(), "Publishing bounding boxes to: %s", bbox_publisher_->get_topic_name());

  // Create the model based on type and load it

  obj_detect_model_ = std::make_unique<rzv_model::YoloxModel>();
  RCLCPP_INFO(this->get_logger(), "Using YoloxModel Detection model");

  // Set model parameters
  if (!class_names_.empty()) {
    obj_detect_model_->set_class_names(class_names_);
  }
  obj_detect_model_->set_confidence_threshold(confidence_threshold_);
  obj_detect_model_->set_iou_threshold(iou_threshold_);

  // Load the model
  if (!obj_detect_model_->load(model_path_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YOLOX model from %s", model_path_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "YOLOX Model %s loaded successfully", model_type_.c_str());
  }
}

YoloXObjectDetection::~YoloXObjectDetection()
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up resources...");
  image_subscription_.reset();
  bbox_publisher_.reset();
  obj_detect_model_.reset();
  diagnostic_timing_publisher_.reset();
}

void YoloXObjectDetection::process_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Quick check - don't process if model not loaded
  if (!obj_detect_model_ || !obj_detect_model_->is_loaded()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Model not loaded, skipping processing");
    return;
  }

  // Log that we received an image (throttled)
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000, "Received image: %dx%d, encoding: %s", msg->width,
    msg->height, msg->encoding.c_str());

  RCLCPP_DEBUG(this->get_logger(), "Processing image in executor thread");

  // Convert the image to YUV422 (YUY2) format which is what our model expects
  cv::Mat image;

  try {
    if (msg->encoding == "bgr8") {
      // Convert BGR to YUV422 (yuv422_yuy2)
      cv::Mat bgr_image(
        msg->height, msg->width, CV_8UC3, const_cast<unsigned char *>(msg->data.data()));
      image = rzv_model::Utils::bgr_to_yuv422(bgr_image, rzv_model::YUV422Format::YUYV);
    } else if (msg->encoding == "rgba8") {
      // Convert RGBA to YUV422 (yuv422_yuy2)
      cv::Mat rgba_image(
        msg->height, msg->width, CV_8UC4, const_cast<unsigned char *>(msg->data.data()));
      image = rzv_model::Utils::rgba_to_yuv422(rgba_image, rzv_model::YUV422Format::YUYV);
    } else if (msg->encoding == "yuv422" || msg->encoding == "yuv422_yuy2") {
      // Already in YUV422_YUY2 format, just create a view (zero-copy)
      image =
        cv::Mat(msg->height, msg->width, CV_8UC2, const_cast<unsigned char *>(msg->data.data()));
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
      return;
    }

    // Run object detection model
    bool has_valid_detections = false;
    auto object_image_input = rzv_model::ModelInput{image, cv::Rect(0, 0, image.cols, image.rows)};
    auto result = obj_detect_model_->run<rzv_model::YOLOXDetectionResult>(object_image_input);

    if (result) {
      // Create pose array for all valid detections
      auto pose_array = std::make_unique<geometry_msgs::msg::PoseArray>();
      pose_array->header.stamp = this->now();
      pose_array->header.frame_id = "camera_frame";

      for (const auto & detection : result->detections) {
        if (detection.is_valid) {
          // Log detection info
          RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Detected %s at: %2d, %2d, %2d, %2d with score %0.2f", detection.class_name.c_str(),
            detection.bbox.x, detection.bbox.y, detection.bbox.width, detection.bbox.height,
            detection.confidence);

          has_valid_detections = true;

          // Add bounding box to the pose array with class label and confidence
          rzv_model::UtilsROS::encode_bounding_box_to_poses(
            *pose_array, detection.bbox, detection.class_name, detection.class_id,
            detection.confidence);
        }
      }

      RCLCPP_DEBUG(this->get_logger(), "Finished processing image");
      // Publish only if we have valid detections
      if (has_valid_detections) {
        bbox_publisher_->publish(std::move(pose_array));

        // Publish diagnostic timing information
        auto diagnostic_msg = rzv_model::UtilsROS::encode_inference_timing_diagnostic(
          "YOLOX Object Detection Inference Timing", result->preprocess_ms, result->inference_ms,
          result->postprocess_ms);
        diagnostic_timing_publisher_->publish(std::move(diagnostic_msg));
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
  }

  RCLCPP_DEBUG(this->get_logger(), "Finished processing image");
}

}  // namespace rzv_object_detection

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create node first to access parameters
  auto node = std::make_shared<rzv_object_detection::YoloXObjectDetection>();

  // Get processing threads from parameter
  int thread_count = node->get_parameter("processing_threads").as_int();

  // Create multi-threaded executor with configured threads
  // Using 1 thread ensures sequential processing similar to original code
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), thread_count);

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
