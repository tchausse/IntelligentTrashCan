// Copyright 2026 Thomas Chausse

// ROS node that reports how full the bin is.

#ifndef TRASHCAN_SENSORS__FILL_LEVEL_NODE_HPP_
#define TRASHCAN_SENSORS__FILL_LEVEL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "trashcan_sensors/fill_level.hpp"

namespace trashcan_sensors
{

// Subscribes:
//   bin/fill_scan   sensor_msgs/LaserScan  raw readings of the in-bin range sensor
// Publishes:
//   bin/fill_range  sensor_msgs/Range      distance to the top of the trash
//   bin/fill_level  std_msgs/Float32       fill level in percent, 0 (empty) to 100 (full)
//   bin/full        std_msgs/Bool          latched; true when the bin needs emptying
// Parameters:
//   empty_range_m           range to the bin floor of an empty bin (required)
//   full_range_m            range to the trash of a 100 % full bin (required)
//   full_threshold_percent  level at which the bin counts as full (default 90)
//   full_hysteresis_percent drop below threshold needed to clear full (default 5)
class FillLevelNode : public rclcpp::Node
{
public:
  explicit FillLevelNode(const rclcpp::NodeOptions & options);

private:
  void on_scan(const sensor_msgs::msg::LaserScan & scan);

  FillLevelCalibration calibration_;
  FullDetector full_detector_;
  std::optional<bool> last_full_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr level_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr full_pub_;
};

}  // namespace trashcan_sensors

#endif  // TRASHCAN_SENSORS__FILL_LEVEL_NODE_HPP_
