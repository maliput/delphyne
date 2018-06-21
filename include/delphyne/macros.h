// Copyright 2017 Toyota Research Institute

#pragma once

#include <string>

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
/// Deletes the special member functions for copy-construction, copy-assignment,
/// move-construction, and move-assignment.
/// Invoke this macro in the public section of the class declaration, e.g.:
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

/// \def STR_SIMPLE
/// Internal stringify a token

/// \def STR
/// Stringify a token

#define STR_SIMPLE(x) #x
#define STR(x) STR_SIMPLE(x)

/// \def DELPHYNE_VALIDATE
/// Used to validate that an argument passed into a function or method is true;
/// if not, an exception of type exctype is thrown.

#define DELPHYNE_VALIDATE(pred, exctype, message)                       \
  do {                                                                  \
    if (!(pred)) {                                                      \
      std::string fullname = std::string(__FILE__);                     \
      size_t found = fullname.find_last_of("/");                        \
      std::string fname = fullname;                                     \
      if (found != std::string::npos) {                                 \
        fname = fullname.substr(found+1);                               \
      }                                                                 \
      std::string errmsg(fname);                                        \
      errmsg.append(":").append(__func__).append(":").append(STR(__LINE__)); \
      errmsg.append(": ").append(message);                              \
      throw exctype(errmsg);                                            \
    }                                                                   \
  } while (0)
