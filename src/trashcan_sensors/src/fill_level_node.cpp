// Copyright 2026 Thomas Chausse

#include "trashcan_sensors/fill_level_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace trashcan_sensors
{

namespace
{

FillLevelCalibration declare_calibration(rclcpp::Node & node)
{
  FillLevelCalibration calibration{
    node.declare_parameter<double>("empty_range_m"),
    node.declare_parameter<double>("full_range_m")};
  validate(calibration);
  return calibration;
}

}  // namespace

FillLevelNode::FillLevelNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fill_level", options),
  calibration_(declare_calibration(*this)),
  full_detector_(
    declare_parameter<double>("full_threshold_percent", 90.0),
    declare_parameter<double>("full_hysteresis_percent", 5.0))
{
  range_pub_ = create_publisher<sensor_msgs::msg::Range>("bin/fill_range", 10);
  level_pub_ = create_publisher<std_msgs::msg::Float32>("bin/fill_level", 10);
  // Latched so that late subscribers, such as a notifier, see the current state.
  full_pub_ = create_publisher<std_msgs::msg::Bool>(
    "bin/full", rclcpp::QoS(1).reliable().transient_local());
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "bin/fill_scan", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan & scan) {on_scan(scan);});
}

void FillLevelNode::on_scan(const sensor_msgs::msg::LaserScan & scan)
{
  const auto range = nearest_range(scan.ranges, scan.range_min, scan.range_max);
  if (!range) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Fill-level sensor sees no surface; is it blocked or out of place?");
    return;
  }

  sensor_msgs::msg::Range range_msg;
  range_msg.header = scan.header;
  range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  range_msg.field_of_view = scan.angle_max - scan.angle_min;
  range_msg.min_range = scan.range_min;
  range_msg.max_range = scan.range_max;
  range_msg.range = static_cast<float>(*range);
  range_pub_->publish(range_msg);

  const double percent = fill_percent(*range, calibration_);
  std_msgs::msg::Float32 level_msg;
  level_msg.data = static_cast<float>(percent);
  level_pub_->publish(level_msg);

  const bool full = full_detector_.update(percent);
  if (full != last_full_) {
    if (full) {
      RCLCPP_INFO(get_logger(), "Bin is full (%.0f %%), it needs emptying", percent);
    }
    std_msgs::msg::Bool full_msg;
    full_msg.data = full;
    full_pub_->publish(full_msg);
    last_full_ = full;
  }
}

}  // namespace trashcan_sensors

RCLCPP_COMPONENTS_REGISTER_NODE(trashcan_sensors::FillLevelNode)
