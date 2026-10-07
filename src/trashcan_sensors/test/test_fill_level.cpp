// Copyright 2026 Thomas Chausse
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <vector>

#include "trashcan_sensors/fill_level.hpp"

namespace trashcan_sensors {
namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

const FillLevelCalibration kCalibration{0.40, 0.05};

TEST(Validate, AcceptsOrderedPositiveRanges) {
  EXPECT_NO_THROW(validate(kCalibration));
}

TEST(Validate, RejectsBadRanges) {
  EXPECT_THROW(validate({0.05, 0.40}), std::invalid_argument);
  EXPECT_THROW(validate({0.40, 0.40}), std::invalid_argument);
  EXPECT_THROW(validate({0.40, 0.0}), std::invalid_argument);
}

TEST(NearestRange, PicksClosestReading) {
  EXPECT_DOUBLE_EQ(*nearest_range_m({0.30F, 0.25F, 0.35F}, 0.03, 1.0), 0.25F);
}

TEST(NearestRange, IgnoresReadingsWithoutReturn) {
  EXPECT_DOUBLE_EQ(*nearest_range_m({kInf, 0.30F, kNan, 2.0F}, 0.03, 1.0),
                   0.30F);
  EXPECT_FALSE(nearest_range_m({kInf, kNan, 2.0F}, 0.03, 1.0).has_value());
  EXPECT_FALSE(nearest_range_m({}, 0.03, 1.0).has_value());
}

TEST(NearestRange, ClampsTooCloseReadingsToMinimum) {
  EXPECT_DOUBLE_EQ(*nearest_range_m({-kInf, 0.30F}, 0.03, 1.0), 0.03);
  EXPECT_DOUBLE_EQ(*nearest_range_m({0.01F}, 0.03, 1.0), 0.03);
}

TEST(FillPercent, MapsCalibrationEndsToZeroAndHundred) {
  EXPECT_DOUBLE_EQ(fill_percent(0.40, kCalibration), 0.0);
  EXPECT_DOUBLE_EQ(fill_percent(0.05, kCalibration), 100.0);
  EXPECT_NEAR(fill_percent(0.225, kCalibration), 50.0, 1e-9);
}

TEST(FillPercent, ClampsOutsideCalibration) {
  EXPECT_DOUBLE_EQ(fill_percent(0.45, kCalibration), 0.0);
  EXPECT_DOUBLE_EQ(fill_percent(0.03, kCalibration), 100.0);
}

TEST(FullDetector, RejectsBadSettings) {
  EXPECT_THROW(FullDetector(0.0, 0.0), std::invalid_argument);
  EXPECT_THROW(FullDetector(101.0, 5.0), std::invalid_argument);
  EXPECT_THROW(FullDetector(90.0, -1.0), std::invalid_argument);
  EXPECT_THROW(FullDetector(90.0, 90.0), std::invalid_argument);
}

TEST(FullDetector, AppliesHysteresis) {
  FullDetector detector(90.0, 5.0);
  EXPECT_FALSE(detector.full());
  EXPECT_FALSE(detector.update(89.0));
  EXPECT_TRUE(detector.update(90.0));
  EXPECT_TRUE(detector.update(86.0));
  EXPECT_TRUE(detector.update(85.0));
  EXPECT_FALSE(detector.update(84.9));
  EXPECT_FALSE(detector.update(89.9));
}

}  // namespace
}  // namespace trashcan_sensors
