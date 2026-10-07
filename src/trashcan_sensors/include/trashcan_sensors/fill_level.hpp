// Copyright 2026 Thomas Chausse
// SPDX-License-Identifier: MIT

#ifndef TRASHCAN_SENSORS__FILL_LEVEL_HPP_
#define TRASHCAN_SENSORS__FILL_LEVEL_HPP_

#include <optional>
#include <vector>

namespace trashcan_sensors {

struct FillLevelCalibration {
  double empty_bin_range_m;
  double full_bin_range_m;
};

void validate(const FillLevelCalibration& calibration);

// The closest reading is the top of the trash pile. Readings below range_min,
// including -inf, are an object right in front of the sensor rather than a
// missing return, so they count as range_min instead of being dropped.
std::optional<double> nearest_range_m(const std::vector<float>& ranges_m,
                                      double range_min_m, double range_max_m);

double fill_percent(double range_m, const FillLevelCalibration& calibration);

// Once full, the bin stays full until the level drops hysteresis_percent below
// the threshold, so sensor noise around the threshold does not toggle the flag.
class FullDetector {
 public:
  FullDetector(double threshold_percent, double hysteresis_percent);

  bool update(double fill_level_percent);
  bool full() const { return full_; }

 private:
  double threshold_percent_;
  double hysteresis_percent_;
  bool full_{false};
};

}  // namespace trashcan_sensors

#endif  // TRASHCAN_SENSORS__FILL_LEVEL_HPP_
