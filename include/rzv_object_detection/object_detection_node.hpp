#pragma once

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "rzv_model/yolox_hand_model.hpp"

namespace rzv_object_detection
{

class ObjectDetection : public rclcpp::Node
{
public:
  explicit ObjectDetection();
  ~ObjectDetection();

private:
  void process_image(const sensor_msgs::msg::Image::SharedPtr msg);

  // Subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr bbox_publisher_;

  // Model(s)
  std::unique_ptr<rzv_model::YoloxModel> obj_detect_model_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::string model_path_;
  std::string model_type_;
  std::vector<std::string> class_names_;
  float confidence_threshold_;
  float iou_threshold_;
};

}  // namespace rzv_object_detection
