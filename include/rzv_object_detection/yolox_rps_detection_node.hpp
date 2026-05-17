// *********************************************************************************************************************
//  Copyright (C) 2026 Renesas Electronics Corporation and/or its licensors.
//  SPDX-License-Identifier: AGPL-3.0-only
// *********************************************************************************************************************
#pragma once

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

#include "rzv_yolox/yolox_model.hpp"

namespace rzv_object_detection
{

class YoloXObjectDetection : public rclcpp::Node
{
public:
  explicit YoloXObjectDetection();
  ~YoloXObjectDetection();

private:
  void process_image(const sensor_msgs::msg::Image::SharedPtr msg);

  // Subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr bbox_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr object_detection_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_timing_publisher_;

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
