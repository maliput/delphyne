// Copyright 2017 Toyota Research Institute

#pragma once

/*****************************************************************************
** Includes
*****************************************************************************/

#include <chrono>

#include <drake/common/autodiff.h>
#include <drake/common/eigen_types.h>
#include <drake/common/symbolic.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

/*****************************************************************************
** Typedefs
*****************************************************************************/

// Drake
using AutoDiff = ::drake::AutoDiffXd;
using Symbolic = ::drake::symbolic::Expression;

// Time
using Duration = std::chrono::duration<double>;
using RealtimeClock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<RealtimeClock, Duration>;

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
