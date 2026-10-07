// Copyright 2026 Thomas Chausse
// SPDX-License-Identifier: MIT

#include "trashcan_sensors/fill_level.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace trashcan_sensors {

namespace {

constexpr double kMaxFillPercent = 100.0;

}  // namespace

void validate(const FillLevelCalibration& calibration) {
  if (!(calibration.full_bin_range_m > 0.0 &&
        calibration.full_bin_range_m < calibration.empty_bin_range_m)) {
    throw std::invalid_argument(
        "fill level calibration needs 0 < full_range < empty_range");
  }
}

std::optional<double> nearest_range_m(const std::vector<float>& ranges_m,
                                      double range_min_m, double range_max_m) {
  std::optional<double> nearest_m;
  for (const float reading_m : ranges_m) {
    const bool has_return = !std::isnan(reading_m) && reading_m <= range_max_m;
    if (!has_return) {
      continue;
    }
    const double range_m =
        std::max(static_cast<double>(reading_m), range_min_m);
    if (!nearest_m || range_m < *nearest_m) {
      nearest_m = range_m;
    }
  }
  return nearest_m;
}

double fill_percent(double range_m, const FillLevelCalibration& calibration) {
  const double fill_fraction =
      (calibration.empty_bin_range_m - range_m) /
      (calibration.empty_bin_range_m - calibration.full_bin_range_m);
  return kMaxFillPercent * std::clamp(fill_fraction, 0.0, 1.0);
}

FullDetector::FullDetector(double threshold_percent, double hysteresis_percent)
    : threshold_percent_(threshold_percent),
      hysteresis_percent_(hysteresis_percent) {
  if (!(threshold_percent > 0.0 && threshold_percent <= kMaxFillPercent)) {
    throw std::invalid_argument("full threshold must be in (0, 100]");
  }
  if (!(hysteresis_percent >= 0.0 && hysteresis_percent < threshold_percent)) {
    throw std::invalid_argument("full hysteresis must be in [0, threshold)");
  }
}

bool FullDetector::update(double fill_level_percent) {
  const double active_threshold_percent =
      full_ ? threshold_percent_ - hysteresis_percent_ : threshold_percent_;
  full_ = fill_level_percent >= active_threshold_percent;
  return full_;
}

}  // namespace trashcan_sensors
