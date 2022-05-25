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

#include <string>

#include <drake/common/drake_assert.h>
#include <drake/common/drake_copyable.h>

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
#define DELPHYNE_ABORT_MESSAGE(msg) DRAKE_DEMAND(msg)

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
#define DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(class) DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(class)

/// \def STR_SIMPLE
/// Internal stringify a token

/// \def STR
/// Stringify a token

#define STR_SIMPLE(x) #x
#ifndef STR
#define STR(x) STR_SIMPLE(x)
#endif

/// \def DELPHYNE_VALIDATE
/// Used to validate that an argument passed into a function or method is true;
/// if not, an exception of type exctype is thrown.

#define DELPHYNE_VALIDATE(pred, exctype, message)                            \
  do {                                                                       \
    if (!(pred)) {                                                           \
      std::string fullname = std::string(__FILE__);                          \
      size_t found = fullname.find_last_of("/");                             \
      std::string fname = fullname;                                          \
      if (found != std::string::npos) {                                      \
        fname = fullname.substr(found + 1);                                  \
      }                                                                      \
      std::string errmsg(fname);                                             \
      errmsg.append(":").append(__func__).append(":").append(STR(__LINE__)); \
      errmsg.append(": ").append(message);                                   \
      throw exctype(errmsg);                                                 \
    }                                                                        \
  } while (0)
