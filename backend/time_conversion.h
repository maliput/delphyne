// Copyright 2017 Toyota Research Institute

#pragma once

#include <cstdint>
#include <utility>

#include <ignition/msgs.hh>

#include "backend/system.h"

namespace delphyne {
namespace backend {

/// @brief Converts from an integer value in microseconds to a
/// pair of integers containing the value in seconds and the
/// remainder of that in nanoseconds.
/// @param[in]  micros An integer value containing the time in
/// microseconds.
/// @param[out] A pair of integers containing the translated
/// time value composed by seconds and nanoseconds.

std::pair<int64_t, int64_t> MicrosToSecsAndNanos(int64_t micros);

/// @brief Converts from an integer value in milliseconds to a
/// pair of integers containing the value in seconds and the
/// remainder of that in nanoseconds.
/// @param[in]  millis An integer value containing the time in
/// milliseconds.
/// @param[out] A pair of integers containing the translated
/// time value composed by seconds and nanoseconds.
std::pair<int64_t, int64_t> MillisToSecsAndNanos(int64_t millis);

/// @brief Converts from a double value in seconds to a pair of
/// integers containing the value in seconds and the remainder
/// of that in nanoseconds.
/// @param[in] time A double containing the time to be translated
/// in seconds.
/// @param[out] A pair of integers containing the translated
/// time value composed by seconds and nanoseconds.
std::pair<int64_t, int64_t> SecsToSecsAndNanos(double time);

/// @brief Converts from a pair of integers containing independent
/// time values in seconds and nanoseconds into a single double in
/// milliseconds.
/// @param[in]  secs An integer value containing the time component
/// in seconds to translate.
/// @param[in]  nsecs An integer value containing the time component
/// in nanoseconds to translate.
/// @param[out] A pair of integers containing the translated
/// time value composed by seconds and nanoseconds.
double SecsAndNanosToMillis(int64_t secs, int64_t nsecs);

/// @brief Generates and returns an ignition::msgs::Time from a given
/// integer value in milliseconds.
/// @param[in]  millis An integer value containing the time in
/// milliseconds.
/// @param[out] An ignition messages Time value composed by the value
/// in seconds and nanoseconds translated from the milliseconds' input.
ignition::msgs::Time MillisToIgnitionTime(int64_t millis);

/// @brief Generates and returns an ignition::msgs::Time from a given
/// integer value in microseconds.
/// @param[in]  millis An integer value containing the time in
/// microseconds.
/// @param[out] An ignition messages Time value composed by the value
/// in seconds and nanoseconds translated from the microseconds' input.
ignition::msgs::Time MicrosToIgnitionTime(int64_t micros);

/// @brief Generates and returns an ignition::msgs::Time from a given
/// double value in seconds.
/// @param[in] time A double containing the time to be translated
/// in seconds.
/// @param[out] An ignition messages Time value composed by the value
/// in seconds and nanoseconds translated from the seconds' double input.
ignition::msgs::Time SecsToIgnitionTime(double secs);

/// @brief Generates and returns an integer value in milliseconds.
/// @param[in] ign_time an ignition messages Time object.
/// @param[out] An integer value in milliseconds representing the
/// total time contained in the ignition message.
int64_t IgnitionTimeToMillis(const ignition::msgs::Time ign_time);

}  // namespace backend
}  // namespace delphyne
