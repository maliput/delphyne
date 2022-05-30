// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2017-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "translations/time_conversion.h"

#include <gtest/gtest.h>
#include <ignition/msgs.hh>

namespace delphyne {

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

}  // namespace delphyne
