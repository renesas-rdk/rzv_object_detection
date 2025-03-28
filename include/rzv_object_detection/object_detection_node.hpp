#pragma once

#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <geometry_msgs/msg/pose_array.hpp>

#include "rzv_model/yolox_pascal_voc_model.hpp"
#include "rzv_model/yolox_hand_model.hpp"
#include "rzv_model/gold_yolox_hand_model.hpp"

namespace rzv_object_detection
{

class ImageProcessor;

class ObjectDetection : public rclcpp::Node
{
public:
  explicit ObjectDetection();
  ~ObjectDetection();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void process_image(sensor_msgs::msg::Image::SharedPtr msg);
  void add_bbox_to_pose_array(geometry_msgs::msg::PoseArray& pose_array, const cv::Rect& bbox);

  // Subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr bbox_publisher_;

  // Model(s)
  std::unique_ptr<rzv_model::YoloxModel> obj_detect_model_;

  std::unique_ptr<ImageProcessor> image_processor_;
  std::string model_path_;
  std::string model_type_;
  std::vector<std::string> class_names_;
  float confidence_threshold_;
  float iou_threshold_;
};

}  // namespace rzv_object_detection
