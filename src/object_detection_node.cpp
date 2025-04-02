#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <unistd.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "rzv_object_detection/object_detection_node.hpp"
#include "rzv_model/utils.hpp"
#include "rzv_model/yolox_pascal_voc_model.hpp"
#include "rzv_model/gold_yolox_hand_model.hpp"

namespace rzv_object_detection
{

ObjectDetection::ObjectDetection() : Node("object_detection")
{
  RCLCPP_INFO(this->get_logger(), "Node object detection started!");

  // Declare parameters with default values
  this->declare_parameter("model_path", "");
  this->declare_parameter("model_type", "yolox_pascal_voc");
  this->declare_parameter("processing_queue_size", 5);
  this->declare_parameter("confidence_threshold", 0.5f);
  this->declare_parameter("iou_threshold", 0.45f);
  this->declare_parameter("class_names", std::vector<std::string>{});
  this->declare_parameter("processing_threads", 1);  // Default to 1 for sequential processing

  // Get parameters
  model_path_ = this->get_parameter("model_path").as_string();
  model_type_ = this->get_parameter("model_type").as_string();
  int queue_size = this->get_parameter("processing_queue_size").as_int();
  confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
  iou_threshold_ = this->get_parameter("iou_threshold").as_double();
  class_names_ = this->get_parameter("class_names").as_string_array();

  // Create a mutually exclusive callback group - ensures one callback runs at a time
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Configure QoS with queue depth and frame dropping policy
  auto qos = rclcpp::QoS(queue_size);
  qos.keep_last(queue_size);  // Only keep latest frames
  qos.reliable();             // Or use best_effort() for more aggressive dropping
  qos.durability_volatile();  // Don't persist old messages

  // Set subscription options with callback group
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  // Create subscription to image topic
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", qos, std::bind(&ObjectDetection::process_image, this, std::placeholders::_1), options);

  // Create publisher for bounding boxes
  bbox_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("bounding_box", 10);

  // Log configuration
  RCLCPP_INFO(this->get_logger(), "ObjectDetection initialized");
  RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Model type: %s", model_type_.c_str());
  RCLCPP_INFO(this->get_logger(), "Image processing queue size: %d", queue_size);
  RCLCPP_INFO(this->get_logger(), "Confidence threshold: %.2f", confidence_threshold_);
  RCLCPP_INFO(this->get_logger(), "IoU threshold: %.2f", iou_threshold_);
  RCLCPP_INFO(this->get_logger(), "Number of classes: %zu", class_names_.size());
  RCLCPP_INFO(this->get_logger(), "Subscribing to image topic: %s", image_subscription_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Publishing bounding boxes to: %s", bbox_publisher_->get_topic_name());

  // Create the model based on type and load it
  if (model_type_ == "gold_yolox_hand")
  {
    obj_detect_model_ = std::make_unique<rzv_model::GoldYoloxHandModel>();
    RCLCPP_INFO(this->get_logger(), "Using YOLOX Hand model");
  }
  else if (model_type_ == "yolox_hand")
  {
    obj_detect_model_ = std::make_unique<rzv_model::YoloxHandModel>();
    RCLCPP_INFO(this->get_logger(), "Using YOLOX Hand model");
  }
  else if (model_type_ == "yolox_pascal_voc")
  {
    obj_detect_model_ = std::make_unique<rzv_model::YoloxPascalVocModel>();
    RCLCPP_INFO(this->get_logger(), "Using YOLOX Pascal VOC model");
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Unrecognized model type: %s, using YOLOX model by default", model_type_.c_str());
    obj_detect_model_ = std::make_unique<rzv_model::YoloxModel>();
  }

  // Set model parameters
  if (!class_names_.empty())
  {
    obj_detect_model_->set_class_names(class_names_);
  }
  obj_detect_model_->set_confidence_threshold(confidence_threshold_);
  obj_detect_model_->set_iou_threshold(iou_threshold_);

  // Load the model
  if (!obj_detect_model_->load(model_path_))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YOLOX model from %s", model_path_.c_str());
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "YOLOX Model %s loaded successfully", model_type_.c_str());
  }
}

ObjectDetection::~ObjectDetection()
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up resources...");
  image_subscription_.reset();
  bbox_publisher_.reset();
  obj_detect_model_.reset();
}

void ObjectDetection::process_image(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Quick check - don't process if model not loaded
  if (!obj_detect_model_ || !obj_detect_model_->is_loaded())
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Model not loaded, skipping processing");
    return;
  }

  // Log that we received an image (throttled)
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received image: %dx%d, encoding: %s", msg->width,
                       msg->height, msg->encoding.c_str());

  RCLCPP_DEBUG(this->get_logger(), "Processing image in executor thread");

  // Convert the image to YUV422 (YUY2) format which is what our model expects
  cv::Mat image;

  try
  {
    if (msg->encoding == "bgr8")
    {
      // Convert BGR to YUV422 (yuv422_yuy2)
      cv::Mat bgr_image(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()));
      image = rzv_model::Utils::bgr_to_yuv422(bgr_image, rzv_model::YUV422Format::YUYV);
    }
    else if (msg->encoding == "rgba8")
    {
      // Convert RGBA to YUV422 (yuv422_yuy2)
      cv::Mat rgba_image(msg->height, msg->width, CV_8UC4, const_cast<unsigned char*>(msg->data.data()));
      image = rzv_model::Utils::rgba_to_yuv422(rgba_image, rzv_model::YUV422Format::YUYV);
    }
    else if (msg->encoding == "yuv422" || msg->encoding == "yuv422_yuy2")
    {
      // Already in YUV422_YUY2 format, just create a view (zero-copy)
      image = cv::Mat(msg->height, msg->width, CV_8UC2, const_cast<unsigned char*>(msg->data.data()));
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
      return;
    }

    // Run object detection model
    auto input = rzv_model::ModelInput{ image, cv::Rect(0, 0, image.cols, image.rows) };
    auto result = obj_detect_model_->run<rzv_model::YOLOXDetectionResult>(input);
    if (result)
    {
      // Create pose array for all valid detections
      auto pose_array = std::make_unique<geometry_msgs::msg::PoseArray>();
      pose_array->header.stamp = this->now();
      pose_array->header.frame_id = "camera_frame";

      bool has_valid_detections = false;

      for (const auto& detection : result->detections)
      {
        if (detection.is_valid)
        {
          has_valid_detections = true;
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Detected %s at: %d, %d, %d, %d with score %0.2f", detection.class_name.c_str(),
                               detection.bbox.x, detection.bbox.y, detection.bbox.width, detection.bbox.height,
                               detection.confidence);

          // Add bounding box to the pose array with class label and confidence
          rzv_model::Utils::encode_bounding_box_to_poses(*pose_array, detection.bbox, detection.class_name,
                                                         detection.class_id, detection.confidence);
        }
      }

      // Publish only if we have valid detections
      if (has_valid_detections)
      {
        bbox_publisher_->publish(std::move(pose_array));
      }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
  }

  RCLCPP_DEBUG(this->get_logger(), "Finished processing image");
}

}  // namespace rzv_object_detection

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Create node first to access parameters
  auto node = std::make_shared<rzv_object_detection::ObjectDetection>();

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
