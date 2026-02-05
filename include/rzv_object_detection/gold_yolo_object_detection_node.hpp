// ********************************************************************************************************************
// Copyright [2025] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
//
// The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
// and/or its licensors ("Renesas") and subject to statutory and contractual protections.
//
// Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
// display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
// purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
// SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
// NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
// INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
// OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
// be subject to different terms.
// ********************************************************************************************************************
#pragma once

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

#include "rzv_gold_yolo/gold_yolo_model.hpp"

namespace rzv_object_detection
{

class GoldYoloObjectDetection : public rclcpp::Node
{
public:
  explicit GoldYoloObjectDetection();
  ~GoldYoloObjectDetection();
private:
  void process_image(const sensor_msgs::msg::Image::SharedPtr msg);

  // Subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr bbox_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_timing_publisher_;

  // Model(s)
  std::unique_ptr<rzv_model::GoldYoloModel> obj_detect_model_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  std::string model_path_;
  std::string model_type_;
  std::vector<std::string> class_names_;
  float confidence_threshold_;
  float iou_threshold_;
};

}  // namespace rzv_object_detection
