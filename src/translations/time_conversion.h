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

#pragma once

#include <cstdint>
#include <utility>

#include <ignition/msgs.hh>

#include "delphyne/macros.h"

namespace delphyne {

/// @brief Converts from an integer value in microseconds to a
/// pair of integers containing the value in seconds and the
/// remainder of that in nanoseconds.
/// @param[in]  micros An integer value containing the time in
/// microseconds.
/// @return A pair of integers containing the translated
/// time value composed by seconds and nanoseconds.

std::pair<int64_t, int64_t> MicrosToSecsAndNanos(int64_t micros);

/// @brief Converts from an integer value in milliseconds to a
/// pair of integers containing the value in seconds and the
/// remainder of that in nanoseconds.
/// @param[in]  millis An integer value containing the time in
/// milliseconds.
/// @return A pair of integers containing the translated
/// time value composed by seconds and nanoseconds.
std::pair<int64_t, int64_t> MillisToSecsAndNanos(int64_t millis);

/// @brief Converts from a double value in seconds to a pair of
/// integers containing the value in seconds and the remainder
/// of that in nanoseconds.
/// @param[in] time A double containing the time to be translated
/// in seconds.
/// @return A pair of integers containing the translated
/// time value composed by seconds and nanoseconds.
std::pair<int64_t, int64_t> SecsToSecsAndNanos(double time);

/// @brief Converts from a pair of integers containing independent
/// time values in seconds and nanoseconds into a single double in
/// milliseconds.
/// @param[in]  secs An integer value containing the time component
/// in seconds to translate.
/// @param[in]  nsecs An integer value containing the time component
/// in nanoseconds to translate.
/// @return A pair of integers containing the translated
/// time value composed by seconds and nanoseconds.
double SecsAndNanosToMillis(int64_t secs, int64_t nsecs);

/// @brief Generates and returns an ignition::msgs::Time from a given
/// integer value in milliseconds.
/// @param[in]  millis An integer value containing the time in
/// milliseconds.
/// @return An ignition messages Time value composed by the value
/// in seconds and nanoseconds translated from the milliseconds' input.
ignition::msgs::Time MillisToIgnitionTime(int64_t millis);

/// @brief Generates and returns an ignition::msgs::Time from a given
/// integer value in microseconds.
/// @param[in] micros An integer value containing the time in
/// microseconds.
/// @return An ignition messages Time value composed by the value
/// in seconds and nanoseconds translated from the microseconds' input.
ignition::msgs::Time MicrosToIgnitionTime(int64_t micros);

/// @brief Generates and returns an ignition::msgs::Time from a given
/// double value in seconds.
/// @param[in] secs A double value containing the time in seconds.
/// @return An ignition messages Time value composed by the value
/// in seconds and nanoseconds translated from the seconds' double input.
ignition::msgs::Time SecsToIgnitionTime(double secs);

/// @brief Generates and returns an integer value in milliseconds.
/// @param[in] ign_time an ignition messages Time object.
/// @return An integer value in milliseconds representing the
/// total time contained in the ignition message.
int64_t IgnitionTimeToMillis(const ignition::msgs::Time ign_time);

}  // namespace delphyne
