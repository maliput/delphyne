// Copyright 2017 Toyota Research Institute

#pragma once

/*****************************************************************************
** Includes
*****************************************************************************/

#include <drake/common/autodiff.h>
#include <drake/common/eigen_types.h>
#include <drake/common/symbolic.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace delphyne {

namespace automotive = drake::automotive;

/*****************************************************************************
** Typedefs
*****************************************************************************/

typedef ::drake::AutoDiffXd AutoDiff;
typedef ::drake::symbolic::Expression Symbolic;

/*****************************************************************************
** Trailers
*****************************************************************************/

}  // namespace delphyne
