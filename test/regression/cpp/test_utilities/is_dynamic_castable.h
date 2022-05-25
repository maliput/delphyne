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
#include <string>

#include <drake/common/nice_type_name.h>
#include <gtest/gtest.h>

namespace delphyne {
namespace test {

/// Checks if @p ptr, a pointer to `FromType` class, can be safely converted to
/// a pointer to `ToType` class. Our motivation is to provide a good diagnostic
/// for what @p ptr _actually_ was when the test fails.
///
/// Here is an illustrative code snippet. We assume that `Prius` and `Camry` are
/// derived classes of `Car`.
///
/// @code
/// const Car* prius = new Prius{};
///
/// // The following assertion fails without diagnostic info.
/// EXPECT_TRUE(dynamic_cast<Camry>(ptr) != nullptr)
///
/// // The following assertion returns `::testing::AssertionFailure()` with
/// // diagnostic info attached.
/// EXPECT_TRUE(is_dynamic_castable<Camry>(prius));
/// // Output:
/// // Value of: is_dynamic_castable<Camry>(prius)
/// // Actual: false (is_dynamic_castable<Camry>(Car* ptr) failed
/// //                because ptr is of dynamic type Prius.)
/// // Expected: true
/// @endcode
template <typename ToType, typename FromType>
::testing::AssertionResult is_dynamic_castable(const FromType* ptr) {
  if (ptr == nullptr) {
    const std::string from_name{drake::NiceTypeName::Get<FromType>()};
    const std::string to_name{drake::NiceTypeName::Get<ToType>()};
    return ::testing::AssertionFailure() << "is_dynamic_castable<" << to_name << ">(" << from_name << "* ptr)"
                                         << " failed because ptr was already nullptr.";
  }
  if (dynamic_cast<const ToType* const>(ptr) == nullptr) {
    const std::string from_name{drake::NiceTypeName::Get<FromType>()};
    const std::string to_name{drake::NiceTypeName::Get<ToType>()};
    const std::string dynamic_name{drake::NiceTypeName::Get(*ptr)};
    return ::testing::AssertionFailure() << "is_dynamic_castable<" << to_name << ">(" << from_name << "* ptr)"
                                         << " failed because ptr is of dynamic type " << dynamic_name << ".";
  }
  return ::testing::AssertionSuccess();
}

/// Checks if @p ptr, a shared pointer to `FromType` class, can be safely
/// converted to a shared pointer to `ToType` class. Our motivation is to
/// provide a good diagnostic for what @p ptr _actually_ was when the test
/// fails.
template <typename ToType, typename FromType>
::testing::AssertionResult is_dynamic_castable(std::shared_ptr<FromType> ptr) {
  return is_dynamic_castable<ToType, FromType>(ptr.get());
}

/// Checks if @p ptr, a shared pointer to `FromType` class, can be safely
/// converted to a shared pointer to `ToType` class. Our motivation is to
/// provide a good diagnostic for what @p ptr _actually_ was when the test
/// fails.
template <typename ToType, typename FromType>
::testing::AssertionResult is_dynamic_castable(const std::unique_ptr<FromType>& ptr) {
  return is_dynamic_castable<ToType, FromType>(ptr.get());
}

}  // namespace test
}  // namespace delphyne
