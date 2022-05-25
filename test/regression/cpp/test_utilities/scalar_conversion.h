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

#include <memory>

#include "is_dynamic_castable.h"

namespace delphyne {
namespace test {

// A helper function incompatible with symbolic::Expression.  This is useful to
// prove that scalar conversion code does not even instantiate this function
// when T = symbolic::Expression when told not to.  If it did, we see compile
// errors that this function does not compile with T = symbolic::Expression.
template <typename T>
static T copysign_int_to_non_symbolic_scalar(int magic, const T& value) {
  if ((magic < 0) == (value < 0.0)) {
    return value;
  } else {
    return -value;
  }
}
}  // namespace test

/// Tests whether the given device under test of type S<double> can be
/// converted to use AutoDiffXd as its scalar type.  If the test passes,
/// additional checks on the converted object of type `const S<AutoDiffXd>&`
/// can be passed via a lambda into @p callback.  The Callback must take an
/// `const S<AutoDiffXd>&` and return void; a typical value would be a lambda
/// such as `[](const auto& converted) { EXPECT_TRUE(converted.thing()); }`.
template <template <typename> class S, typename Callback>
::testing::AssertionResult is_autodiffxd_convertible(const S<double>& dut, Callback callback) {
  // We must use salted local variable names ("_67273" suffix) to work around
  // GCC 5.4 bug https://gcc.gnu.org/bugzilla/show_bug.cgi?id=67273 because the
  // `callback` is a generic lambda.  The bug is fixed as of GCC 6.1.

  // Check if a proper type came out; return early if not.
  std::unique_ptr<drake::systems::System<drake::AutoDiffXd>> converted_67273 = dut.ToAutoDiffXdMaybe();
  ::testing::AssertionResult result_67273 = delphyne::test::is_dynamic_castable<S<drake::AutoDiffXd>>(converted_67273);
  if (!result_67273) {
    return result_67273;
  }

  // Allow calling code to specify additional tests on the converted System.
  const S<drake::AutoDiffXd>& downcast_67273 = dynamic_cast<const S<drake::AutoDiffXd>&>(*converted_67273);
  callback(downcast_67273);

  return ::testing::AssertionSuccess();
}

/// Tests whether the given device under test of type S<double> can be
/// converted to use AutoDiffXd as its scalar type.
template <template <typename> class S>
::testing::AssertionResult is_autodiffxd_convertible(const S<double>& dut) {
  return is_autodiffxd_convertible(dut, [](const auto&) {});
}

/// Tests whether the given device under test of type S<double> can be
/// converted to use Expression as its scalar type.  If the test passes,
/// additional checks on the converted object of type `const S<Expression>&`
/// can be passed via a lambda into @p callback.  The Callback must take an
/// `const S<Expression>&` and return void; a typical value would be a lambda
/// such as `[](const auto& converted) { EXPECT_TRUE(converted.thing()); }`.
template <template <typename> class S, typename Callback>
::testing::AssertionResult is_symbolic_convertible(const S<double>& dut, Callback callback) {
  // We must use salted local variable names ("_67273" suffix) to work around
  // GCC 5.4 bug https://gcc.gnu.org/bugzilla/show_bug.cgi?id=67273 because the
  // `callback` is a generic lambda.  The bug is fixed as of GCC 6.1.

  // Check if a proper type came out; return early if not.
  std::unique_ptr<drake::systems::System<drake::symbolic::Expression>> converted_67273 = dut.ToSymbolicMaybe();
  ::testing::AssertionResult result_67273 =
      delphyne::test::is_dynamic_castable<S<drake::symbolic::Expression>>(converted_67273);
  if (!result_67273) {
    return result_67273;
  }

  // Allow calling code to specify additional tests on the converted System.
  const S<drake::symbolic::Expression>& downcast_67273 =
      dynamic_cast<const S<drake::symbolic::Expression>&>(*converted_67273);
  callback(downcast_67273);

  return ::testing::AssertionSuccess();
}

/// Tests whether the given device under test of type S<double> can be
/// converted to use Expression as its scalar type.
template <template <typename> class S>
::testing::AssertionResult is_symbolic_convertible(const S<double>& dut) {
  return is_symbolic_convertible(dut, [](const auto&) {});
}

}  // namespace delphyne
