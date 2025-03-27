#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <unistd.h>
#include <geometry_msgs/msg/pose.hpp>
#include "rzv_object_detection/object_detection_node.hpp"
#include "rzv_model/utils.hpp"

namespace rzv_object_detection
{

class ImageProcessor
{
public:
  ImageProcessor(int max_queue_size = 5) : running_(true), max_queue_size_(max_queue_size)
  {
    processing_thread_ = std::thread(&ImageProcessor::process_loop, this);
  }

  ~ImageProcessor()
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      running_ = false;
      condition_.notify_all();
    }
    if (processing_thread_.joinable())
    {
      processing_thread_.join();
    }
  }

  // Add an image to the processing queue
  bool add_task(sensor_msgs::msg::Image::SharedPtr msg,
                std::function<void(sensor_msgs::msg::Image::SharedPtr)> process_func)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // Check if queue is full
    if (tasks_.size() >= static_cast<size_t>(max_queue_size_))
    {
      // Drop oldest image and log this
      tasks_.pop();
      dropped_frames_++;
      if (dropped_frames_ % 10 == 1)
      {  // Log only every 10th drop to avoid flooding
        // Note: Can't use RCLCPP_WARN here since we're outside the node class
        std::cerr << "Warning: Processing queue full, dropped " << dropped_frames_
                  << " frames so far. Consider increasing queue size." << std::endl;
      }
    }

    tasks_.push(std::make_pair(msg, process_func));
    condition_.notify_one();
    return true;
  }

private:
  void process_loop()
  {
    while (running_)
    {
      std::function<void(sensor_msgs::msg::Image::SharedPtr)> process_func;
      sensor_msgs::msg::Image::SharedPtr msg;

      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        condition_.wait(lock, [this] { return !tasks_.empty() || !running_; });

        if (!running_)
        {
          // Process remaining tasks before exiting if requested
          if (tasks_.empty())
            break;
        }

        if (!tasks_.empty())
        {
          auto task = tasks_.front();
          msg = task.first;
          process_func = task.second;
          tasks_.pop();
        }
      }

      // Process the image outside the lock
      if (msg && process_func)
      {
        try
        {
          process_func(msg);
        }
        catch (const std::exception& e)
        {
          std::cerr << "Exception in image processing: " << e.what() << std::endl;
        }
      }
    }
  }

  std::thread processing_thread_;
  std::mutex queue_mutex_;
  std::condition_variable condition_;
  std::queue<std::pair<sensor_msgs::msg::Image::SharedPtr, std::function<void(sensor_msgs::msg::Image::SharedPtr)>>>
      tasks_;
  std::atomic<bool> running_;
  int max_queue_size_;
  int dropped_frames_ = 0;
};

ObjectDetection::ObjectDetection() : Node("object_detection")
{
  RCLCPP_INFO(this->get_logger(), "Node object detection started!");

  // Declare parameters with default values
  this->declare_parameter("model_path", "");
  this->declare_parameter("model_type", "yolox");
  this->declare_parameter("processing_queue_size", 5);
  this->declare_parameter("confidence_threshold", 0.5f);
  this->declare_parameter("iou_threshold", 0.45f);
  this->declare_parameter("class_names", std::vector<std::string>{});

  // Get parameters
  model_path_ = this->get_parameter("model_path").as_string();
  model_type_ = this->get_parameter("model_type").as_string();
  int queue_size = this->get_parameter("processing_queue_size").as_int();
  confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
  iou_threshold_ = this->get_parameter("iou_threshold").as_double();
  class_names_ = this->get_parameter("class_names").as_string_array();

  // Initialize image processor
  image_processor_ = std::make_unique<ImageProcessor>(queue_size);

  // Create subscription to image topic
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, std::bind(&ObjectDetection::image_callback, this, std::placeholders::_1));

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
  RCLCPP_INFO(this->get_logger(), "Subscribing to image topic: %s", subscription_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Publishing bounding boxes to: %s", bbox_publisher_->get_topic_name());

  // Create the model based on type and load it
  if (model_type_ == "yolox_hand")
  {
    obj_detect_model_ = std::make_unique<rzv_model::YOLOXHandModel>();
    RCLCPP_INFO(this->get_logger(), "Using YOLOX Hand model");
  }
  else
  {
    // Default to standard YOLOX model
    obj_detect_model_ = std::make_unique<rzv_model::YOLOXModel>();
    RCLCPP_INFO(this->get_logger(), "Using standard YOLOX model");
  }

  // Set model parameters
  if (!class_names_.empty())
  {
    obj_detect_model_->set_class_names(class_names_);
  }
  obj_detect_model_->set_confidence_threshold(confidence_threshold_);
  obj_detect_model_->set_iou_threshold(iou_threshold_);

  if (!obj_detect_model_->load(model_path_))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YOLOX model from %s", model_path_.c_str());
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "YOLOX model loaded successfully");
  }
}

ObjectDetection::~ObjectDetection()
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up resources...");
  subscription_.reset();
  obj_detect_model_.reset();
  bbox_publisher_.reset();
  image_processor_.reset();
}

void ObjectDetection::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Quick check - don't even queue if model not loaded
  if (!obj_detect_model_ || !obj_detect_model_->is_loaded())
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Model not loaded, skipping processing");
    return;
  }

  // Log that we received an image (throttled)
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received image: %dx%d, encoding: %s", msg->width,
                       msg->height, msg->encoding.c_str());

  // Add task to processing queue - use shared_ptr to avoid copying
  image_processor_->add_task(msg, [this](sensor_msgs::msg::Image::SharedPtr msg) { this->process_image(msg); });
}

void ObjectDetection::process_image(sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Processing image in dedicated thread");

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
    auto result = obj_detect_model_->run(input);

    // Output the results
    if (result)
    {
      auto detection_result = dynamic_cast<rzv_model::YOLOXDetectionResult*>(result.get());
      if (detection_result)
      {
        // Create pose array for all valid detections
        auto pose_array = std::make_unique<geometry_msgs::msg::PoseArray>();
        pose_array->header.stamp = this->now();
        pose_array->header.frame_id = "camera_frame";

        bool has_valid_detections = false;

        for (const auto& detection : detection_result->detections)
        {
          if (detection.is_valid)
          {
            has_valid_detections = true;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Detected %s at: %d, %d, %d, %d with score %0.2f", detection.class_name.c_str(),
                                 detection.bbox.x, detection.bbox.y, detection.bbox.width, detection.bbox.height,
                                 detection.confidence);

            // Add bounding box to the pose array
            add_bbox_to_pose_array(*pose_array, detection.bbox);
          }
        }

        // Publish only if we have valid detections
        if (has_valid_detections)
        {
          bbox_publisher_->publish(std::move(pose_array));
        }
      }
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
  }

  RCLCPP_DEBUG(this->get_logger(), "Finished processing image");
}

void ObjectDetection::add_bbox_to_pose_array(geometry_msgs::msg::PoseArray& pose_array, const cv::Rect& bbox)
{
  float x1 = static_cast<float>(bbox.x);
  float y1 = static_cast<float>(bbox.y);
  float x2 = static_cast<float>(bbox.x + bbox.width);
  float y2 = static_cast<float>(bbox.y + bbox.height);
  float z1 = 0.0f;  // Front plane
  float z2 = 0.0f;  // Back plane (same as front for 2D bbox)

  // Bottom face (z=0)
  std::vector<std::array<float, 3>> corners = {
    { x1, y1, z1 },  // 0: bottom-left-front
    { x2, y1, z1 },  // 1: bottom-right-front
    { x2, y2, z1 },  // 2: bottom-right-back
    { x1, y2, z1 },  // 3: bottom-left-back

    // Top face (z=0 for 2D bbox, would be z2 for 3D)
    { x1, y1, z2 },  // 4: top-left-front
    { x2, y1, z2 },  // 5: top-right-front
    { x2, y2, z2 },  // 6: top-right-back
    { x1, y2, z2 }   // 7: top-left-back
  };

  // Add each corner as a pose
  for (const auto& corner : corners)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = corner[0];
    pose.position.y = corner[1];
    pose.position.z = corner[2];
    pose.orientation.w = 1.0;  // Identity quaternion
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;

    pose_array.poses.push_back(pose);
  }
}

}  // namespace rzv_object_detection

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<rzv_object_detection::ObjectDetection>();
    rclcpp::spin(node);
    node.reset();
  }
  rclcpp::shutdown();
  return 0;
}
