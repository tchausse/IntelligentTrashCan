// Copyright 2026 Thomas Chausse
// SPDX-License-Identifier: MIT

#ifndef TRASHCAN_SENSORS__FILL_LEVEL_NODE_HPP_
#define TRASHCAN_SENSORS__FILL_LEVEL_NODE_HPP_

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "trashcan_sensors/fill_level.hpp"

namespace trashcan_sensors {

class FillLevelNode : public rclcpp::Node {
 public:
  explicit FillLevelNode(const rclcpp::NodeOptions& options);

 private:
  void on_scan(const sensor_msgs::msg::LaserScan& scan);
  void publish_range(const sensor_msgs::msg::LaserScan& scan, double range_m);
  void publish_fill_level(double fill_level_percent);
  void publish_full_if_changed(bool full, double fill_level_percent);

  FillLevelCalibration calibration_;
  FullDetector full_detector_;
  std::optional<bool> last_published_full_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fill_level_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr full_pub_;
};

}  // namespace trashcan_sensors

#endif  // TRASHCAN_SENSORS__FILL_LEVEL_NODE_HPP_
