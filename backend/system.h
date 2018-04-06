// Copyright 2017 Toyota Research Institute

#pragma once

#include "drake/common/drake_assert.h"

#if defined(__GNUC__)
#define DELPHYNE_BACKEND_DEPRECATED(version) __attribute__((deprecated))
#define DELPHYNE_BACKEND_FORCEINLINE __attribute__((always_inline))
#elif defined(_WIN32)
#define DELPHYNE_BACKEND_DEPRECATED(version) ()
#define DELPHYNE_BACKEND_FORCEINLINE __forceinline
#else
#define DELPHYNE_BACKEND_DEPRECATED(version) ()
#define DELPHYNE_BACKEND_FORCEINLINE
#endif

/// \def DELPHYNE_ASSERT
/// Used to declare an assertion. Will quit execution otherwise.

/// \def DELPHYNE_DEMAND
/// Used to declare a demand. Will quit execution otherwise.

#define DELPHYNE_ASSERT(condition) DRAKE_ASSERT(condition)
#define DELPHYNE_DEMAND(condition) DRAKE_DEMAND(condition)
#define DELPHYNE_ABORT() DRAKE_ABORT()
