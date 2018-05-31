// Copyright 2018 Toyota Research Institute

#pragma once

#include <chrono>

#include "backend/delphyne_duration.h"
#include "backend/delphyne_realtime_clock.h"

/// @file
/// Provides delphyne::TimePoint as an alias for
/// std::chrono::time_point<RealtimeClock, Duration>
namespace delphyne {

using TimePoint = std::chrono::time_point<RealtimeClock, Duration>;

}  // namespace delphyne
