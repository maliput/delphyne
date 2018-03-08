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

/// \def DELPHYNE_BACKEND_VISIBLE
/// Use to represent "symbol visible" if supported

/// \def DELPHYNE_BACKEND_HIDDEN
/// Use to represent "symbol hidden" if supported
#if defined _WIN32 || defined __CYGWIN__
#ifdef BUILDING_DLL
#ifdef __GNUC__
#define DELPHYNE_BACKEND_VISIBLE __attribute__((dllexport))
#else
#define DELPHYNE_BACKEND_VISIBLE __declspec(dllexport)
#endif
#else
#ifdef __GNUC__
#define DELPHYNE_BACKEND_VISIBLE __attribute__((dllimport))
#else
#define DELPHYNE_BACKEND_VISIBLE __declspec(dllimport)
#endif
#endif
#define DELPHYNE_BACKEND_HIDDEN
#else
#if __GNUC__ >= 4
#define DELPHYNE_BACKEND_VISIBLE __attribute__((visibility("default")))
#define DELPHYNE_BACKEND_HIDDEN __attribute__((visibility("hidden")))
#else
#define DELPHYNE_BACKEND_VISIBLE
#define DELPHYNE_BACKEND_HIDDEN
#endif
#endif

/// \def DELPHYNE_ASSERT
/// Used to declare an assertion. Will quit execution otherwise.

/// \def DELPHYNE_DEMAND
/// Used to declare a demand. Will quit execution otherwise.

#define DELPHYNE_ASSERT(condition) DRAKE_ASSERT(condition)
#define DELPHYNE_DEMAND(condition) DRAKE_DEMAND(condition)
#define DELPHYNE_ABORT() DRAKE_ABORT()
