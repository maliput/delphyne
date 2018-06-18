// Copyright 2017 Toyota Research Institute

#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

#if defined(__GNUC__)
#define DELPHYNE_DEPRECATED(version) __attribute__((deprecated))
#define DELPHYNE_FORCEINLINE __attribute__((always_inline))
#elif defined(_WIN32)
#define DELPHYNE_DEPRECATED(version) ()
#define DELPHYNE_FORCEINLINE __forceinline
#else
#define DELPHYNE_DEPRECATED(version) ()
#define DELPHYNE_FORCEINLINE
#endif

/// \def DELPHYNE_ASSERT
/// Used to declare an assertion. Will quit execution otherwise.

/// \def DELPHYNE_DEMAND
/// Used to declare a demand. Will quit execution otherwise.

#define DELPHYNE_ASSERT(condition) DRAKE_ASSERT(condition)
#define DELPHYNE_DEMAND(condition) DRAKE_DEMAND(condition)
#define DELPHYNE_ABORT() DRAKE_ABORT()

/// \def DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN
/// deletes the special member functions for copy-construction, copy-assignment,
/// move-construction, and move-assignment.
/// Invoke this this macro in the public section of the
/// class declaration, e.g.:
/// <pre>
/// class Foo {
///  public:
///   DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(Foo)
///
///   // ...
/// };
/// </pre>
/// */
#define DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(class) \
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(class)
