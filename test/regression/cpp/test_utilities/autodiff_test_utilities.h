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

#include <drake/common/autodiff.h>

#include "test_utilities/eigen_matrix_compare.h"

namespace delphyne {

using drake::AutoDiffXd;

namespace test {

// Helper to set the derivatives to the desired n-dimensional values, resizing
// its derivatives vector according to the size of `desired`.
inline void SetDerivatives(AutoDiffXd* variable, const Eigen::VectorXd& desired) { variable->derivatives() = desired; }

inline void SetDerivatives(double*, const Eigen::VectorXd&) {}

// Helper to check that the derivatives match, to within the given tolerance,
// the provided expected values.
inline void CheckDerivatives(const AutoDiffXd& variable, const Eigen::VectorXd& expected, const double tol = 1e-9) {
  if (expected.isZero() && (variable.derivatives().size() == 0)) {
    // Zero and empty are functionally equivalent.
    return;
  }
  EXPECT_TRUE(CompareMatrices(expected, variable.derivatives(), tol));
}

inline void CheckDerivatives(const double&, const Eigen::VectorXd&) {}

// Helper to check positivity of the i-th derivative of the variable.
inline void CheckDerivativePositivity(int i, const AutoDiffXd& variable) {
  DRAKE_DEMAND(i >= 0);
  DRAKE_DEMAND(i < variable.derivatives().size());
  EXPECT_LT(0., variable.derivatives()(i));
}

inline void CheckDerivativePositivity(int, const double&) {}

// Helper to check negativity of the i-th derivative of the variable.
inline void CheckDerivativeNegativity(int i, const AutoDiffXd& variable) {
  DRAKE_DEMAND(i >= 0);
  DRAKE_DEMAND(i < variable.derivatives().size());
  EXPECT_GT(0., variable.derivatives()(i));
}

inline void CheckDerivativeNegativity(int, const double&) {}

}  // namespace test
}  // namespace delphyne
