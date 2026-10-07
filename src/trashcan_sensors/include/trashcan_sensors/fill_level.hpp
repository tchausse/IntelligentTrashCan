// Copyright 2026 Thomas Chausse

// Bin fill-level math, kept free of ROS so it can be unit tested directly.

#ifndef TRASHCAN_SENSORS__FILL_LEVEL_HPP_
#define TRASHCAN_SENSORS__FILL_LEVEL_HPP_

#include <optional>
#include <vector>

namespace trashcan_sensors
{

// Ranges measured by the fill-level sensor at the two ends of the scale.
struct FillLevelCalibration
{
  // Range to the bin floor when the bin is empty.
  double empty_range;
  // Range to the trash surface when the bin counts as 100 % full.
  double full_range;
};

// Throws std::invalid_argument unless 0 < full_range < empty_range.
void validate(const FillLevelCalibration & calibration);

// Returns the closest reading of a range scan, which is the top of the trash
// pile. Readings below range_min (including -inf) mean an object right in front
// of the sensor and count as range_min. Readings with no return (+inf, NaN or
// beyond range_max) are ignored. Returns nullopt when no reading is usable.
std::optional<double> nearest_range(
  const std::vector<float> & ranges, double range_min, double range_max);

// Converts a range into a fill percentage, clamped to [0, 100].
double fill_percent(double range, const FillLevelCalibration & calibration);

// Turns the fill percentage into a full / not full flag. Once full, the bin
// stays full until the level drops below threshold - hysteresis, so that noise
// around the threshold does not toggle the flag.
class FullDetector
{
public:
  // Throws std::invalid_argument unless 0 < threshold <= 100 and
  // 0 <= hysteresis < threshold.
  FullDetector(double threshold_percent, double hysteresis_percent);

  bool update(double percent);
  bool full() const {return full_;}

private:
  double threshold_percent_;
  double hysteresis_percent_;
  bool full_{false};
};

}  // namespace trashcan_sensors

#endif  // TRASHCAN_SENSORS__FILL_LEVEL_HPP_
