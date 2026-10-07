// Copyright 2026 Thomas Chausse
// SPDX-License-Identifier: MIT

#include "trashcan_sensors/fill_level_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace trashcan_sensors {

namespace {

constexpr double kDefaultFullThresholdPercent = 90.0;
constexpr double kDefaultFullHysteresisPercent = 5.0;
constexpr size_t kPublisherQueueDepth = 10;
constexpr int kNoSurfaceWarningPeriodMs = 10000;

FillLevelCalibration declare_calibration(rclcpp::Node& node) {
  const FillLevelCalibration calibration{
      node.declare_parameter<double>("empty_range_m"),
      node.declare_parameter<double>("full_range_m")};
  validate(calibration);
  return calibration;
}

}  // namespace

FillLevelNode::FillLevelNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("fill_level", options),
      calibration_(declare_calibration(*this)),
      full_detector_(declare_parameter<double>("full_threshold_percent",
                                               kDefaultFullThresholdPercent),
                     declare_parameter<double>("full_hysteresis_percent",
                                               kDefaultFullHysteresisPercent)) {
  range_pub_ = create_publisher<sensor_msgs::msg::Range>("bin/fill_range",
                                                         kPublisherQueueDepth);
  fill_level_pub_ = create_publisher<std_msgs::msg::Float32>(
      "bin/fill_level", kPublisherQueueDepth);
  // Latched so that late subscribers, such as a notifier, see the current
  // state.
  full_pub_ = create_publisher<std_msgs::msg::Bool>(
      "bin/full", rclcpp::QoS(1).reliable().transient_local());
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "bin/fill_scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan& scan) { on_scan(scan); });
}

void FillLevelNode::on_scan(const sensor_msgs::msg::LaserScan& scan) {
  const auto range_m =
      nearest_range_m(scan.ranges, scan.range_min, scan.range_max);
  if (!range_m) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), kNoSurfaceWarningPeriodMs,
        "Fill-level sensor sees no surface; is it blocked or out of place?");
    return;
  }

  publish_range(scan, *range_m);
  const double fill_level_percent = fill_percent(*range_m, calibration_);
  publish_fill_level(fill_level_percent);
  publish_full_if_changed(full_detector_.update(fill_level_percent),
                          fill_level_percent);
}

void FillLevelNode::publish_range(const sensor_msgs::msg::LaserScan& scan,
                                  double range_m) {
  sensor_msgs::msg::Range range_msg;
  range_msg.header = scan.header;
  range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  range_msg.field_of_view = scan.angle_max - scan.angle_min;
  range_msg.min_range = scan.range_min;
  range_msg.max_range = scan.range_max;
  range_msg.range = static_cast<float>(range_m);
  range_pub_->publish(range_msg);
}

void FillLevelNode::publish_fill_level(double fill_level_percent) {
  std_msgs::msg::Float32 fill_level_msg;
  fill_level_msg.data = static_cast<float>(fill_level_percent);
  fill_level_pub_->publish(fill_level_msg);
}

void FillLevelNode::publish_full_if_changed(bool full,
                                            double fill_level_percent) {
  if (full == last_published_full_) {
    return;
  }
  if (full) {
    RCLCPP_INFO(get_logger(), "Bin is full (%.0f %%), it needs emptying",
                fill_level_percent);
  }
  std_msgs::msg::Bool full_msg;
  full_msg.data = full;
  full_pub_->publish(full_msg);
  last_published_full_ = full;
}

}  // namespace trashcan_sensors

RCLCPP_COMPONENTS_REGISTER_NODE(trashcan_sensors::FillLevelNode)
