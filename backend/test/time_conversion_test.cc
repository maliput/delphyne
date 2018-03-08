// Copyright 2017 Toyota Research Institute

#include "backend/time_conversion.h"

#include "gtest/gtest.h"

#include <ignition/msgs.hh>

namespace delphyne {
namespace backend {

// @brief Asserts that a given time value in microseconds is
// correctly translated into a pair of values in seconds and
// nanoseconds, whose aditive value is equivalent to the original.
GTEST_TEST(TimeConversionTest, MicrosToSecsAndNanosTest) {
  const int64_t time_micros(123456789);
  const std::pair<int64_t, int64_t> result(MicrosToSecsAndNanos(time_micros));
  EXPECT_EQ(123, std::get<0>(result));
  EXPECT_EQ(456789000, std::get<1>(result));
}

// @brief Asserts that a given time value in milliseconds is
// correctly translated into a pair of values in seconds and
// nanoseconds, whose aditive value is equivalent to the original.
GTEST_TEST(TimeConversionTest, MillisToSecsAndNanosTest) {
  const int64_t time_millis(123456789);
  const std::pair<int64_t, int64_t> result(MillisToSecsAndNanos(time_millis));
  EXPECT_EQ(123456, std::get<0>(result));
  EXPECT_EQ(789000000, std::get<1>(result));
}

// @brief Asserts that a given pair of time values in seconds
// and nanoseconds are correctly translated into a single value
// in milliseconds.
GTEST_TEST(TimeConversionTest, SecsAndNanosToMillisTest) {
  const int64_t secs(123456);
  const int64_t nanos(789000000);
  const double time_millis(SecsAndNanosToMillis(secs, nanos));
  EXPECT_EQ(123456789, time_millis);
}

// @brief Asserts that a given double value in seconds
// is correctly translated into a pair of values in seconds
// and nanoseconds, respectivelly.
GTEST_TEST(TimeConversionTest, SecsToSecsAndNanosTest) {
  const double time_secs(123.456789);
  const std::pair<int64_t, int64_t> result(SecsToSecsAndNanos(time_secs));
  EXPECT_EQ(123, std::get<0>(result));
  EXPECT_EQ(456789000, std::get<1>(result));
}

// @brief Asserts that a given integer value in milliseconds is
// correctly translated into an ignition::msgs::Time value.
GTEST_TEST(TimeConversionTest, MillisToIgnitionTimeTest) {
  const int64_t time_millis(123456789);
  const ignition::msgs::Time ign_time(MillisToIgnitionTime(time_millis));
  EXPECT_EQ(123456, ign_time.sec());
  EXPECT_EQ(789000000, ign_time.nsec());
}

// @brief Asserts that a given integer value in milliseconds is
// correctly translated into an ignition::msgs::Time value.
GTEST_TEST(TimeConversionTest, MicrosToIgnitionTimeTest) {
  const int64_t time_micros(123456789);
  const ignition::msgs::Time ign_time(MicrosToIgnitionTime(time_micros));
  EXPECT_EQ(123, ign_time.sec());
  EXPECT_EQ(456789000, ign_time.nsec());
}

// @brief Asserts that a given double value in seconds is
// correctly translated into an ignition::msgs::Time value.
GTEST_TEST(TimeConversionTest, SecsToIgnitionTimeTest) {
  const double time_secs(123.456789);
  const ignition::msgs::Time ign_time(SecsToIgnitionTime(time_secs));
  EXPECT_EQ(123, ign_time.sec());
  EXPECT_EQ(456789000, ign_time.nsec());
}

GTEST_TEST(TimeConversionTest, IgnitionTimeToMillisTest) {
  ignition::msgs::Time ign_time;
  ign_time.set_sec(123456);
  ign_time.set_nsec(789000000);
  EXPECT_EQ(123456789, IgnitionTimeToMillis(ign_time));
}

}  // namespace backend
}  // namespace delphyne
