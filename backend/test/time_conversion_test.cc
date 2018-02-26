// Copyright 2017 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "backend/time_conversion.h"

#include "gtest/gtest.h"

namespace delphyne {
namespace backend {

// @brief Asserts that a given time value in microseconds is
// correctly converted into a pair of values in seconds and
// nanoseconds, whose aditive value is equivalent to the original.
GTEST_TEST(TimeConversionTest, MicrosToSecsAndNanosTest) {
  const int64_t time_micros(123456789);
  const std::pair<int64_t, int64_t> result(MicrosToSecsAndNanos(time_micros));
  EXPECT_EQ(123, std::get<0>(result));
  EXPECT_EQ(456789000, std::get<1>(result));
}

// @brief Asserts that a given time value in milliseconds is
// correctly converted into a pair of values in seconds and
// nanoseconds, whose aditive value is equivalent to the original.
GTEST_TEST(TimeConversionTest, MillisToSecsAndNanosTest) {
  const int64_t time_micros(123456789);
  const std::pair<int64_t, int64_t> result(MillisToSecsAndNanos(time_micros));
  EXPECT_EQ(123456, std::get<0>(result));
  EXPECT_EQ(789000000, std::get<1>(result));
}

// @brief Asserts that a given pair of time values in seconds an
// nanoseconds are correctly converted into a single value in milliseconds.
GTEST_TEST(TimeConversionTest, SecsAndNanosToMillisTest) {
  const int64_t secs(123456);
  const int64_t nanos(789000000);
  const double time_millis(SecsAndNanosToMillis(secs, nanos));
  EXPECT_EQ(123456789, time_millis);
}

// @brief Asserts that a given decimal time value in seconds
// is correctly converted into a pair of values in seconds and
// nanoseconds, respectivelly.
GTEST_TEST(TimeConversionTest, ToSecsAndNanosTest) {
  const double time_secs(123.456789);
  const std::pair<int64_t, int64_t> result(ToSecsAndNanos(time_secs));
  EXPECT_EQ(123, std::get<0>(result));
  EXPECT_EQ(456789000, std::get<1>(result));
}

}  // namespace backend
}  // namespace delphyne
