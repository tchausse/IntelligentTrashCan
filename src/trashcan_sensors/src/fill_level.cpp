// Copyright 2026 Thomas Chausse

#include "trashcan_sensors/fill_level.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace trashcan_sensors
{

void validate(const FillLevelCalibration & calibration)
{
  if (!(calibration.full_range > 0.0 && calibration.full_range < calibration.empty_range)) {
    throw std::invalid_argument(
            "fill level calibration needs 0 < full_range < empty_range");
  }
}

std::optional<double> nearest_range(
  const std::vector<float> & ranges, double range_min, double range_max)
{
  std::optional<double> nearest;
  for (const float reading : ranges) {
    if (std::isnan(reading) || reading > range_max) {
      continue;
    }
    const double range = std::max(static_cast<double>(reading), range_min);
    if (!nearest || range < *nearest) {
      nearest = range;
    }
  }
  return nearest;
}

double fill_percent(double range, const FillLevelCalibration & calibration)
{
  const double fraction =
    (calibration.empty_range - range) / (calibration.empty_range - calibration.full_range);
  return 100.0 * std::clamp(fraction, 0.0, 1.0);
}

FullDetector::FullDetector(double threshold_percent, double hysteresis_percent)
: threshold_percent_(threshold_percent), hysteresis_percent_(hysteresis_percent)
{
  if (!(threshold_percent > 0.0 && threshold_percent <= 100.0)) {
    throw std::invalid_argument("full threshold must be in (0, 100]");
  }
  if (!(hysteresis_percent >= 0.0 && hysteresis_percent < threshold_percent)) {
    throw std::invalid_argument("full hysteresis must be in [0, threshold)");
  }
}

bool FullDetector::update(double percent)
{
  if (full_) {
    full_ = percent >= threshold_percent_ - hysteresis_percent_;
  } else {
    full_ = percent >= threshold_percent_;
  }
  return full_;
}

}  // namespace trashcan_sensors
