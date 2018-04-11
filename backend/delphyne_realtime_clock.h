// Copyright 2018 Toyota Research Institute

#pragma once

#include <chrono>

/// @file
/// Provides delphyne::RealtimeClock as an alias for std::chrono::steady_clock.

namespace delphyne {

using RealtimeClock = std::chrono::steady_clock;

}  // namespace delphyne
