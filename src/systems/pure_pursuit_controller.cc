// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
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

#include "systems/pure_pursuit_controller.h"

#include <cmath>

#include <drake/common/default_scalars.h>
#include <drake/common/drake_assert.h>

#include "systems/pure_pursuit.h"

namespace delphyne {

using drake::systems::BasicVector;
using drake::systems::rendering::PoseVector;

static constexpr int kPpParamsIndex{0};
static constexpr int kCarParamsIndex{1};

template <typename T>
PurePursuitController<T>::PurePursuitController()
    : drake::systems::LeafSystem<T>(drake::systems::SystemTypeTag<::delphyne::PurePursuitController>{}),
      lane_index_{
          this->DeclareAbstractInputPort(drake::systems::kUseDefaultName, drake::Value<LaneDirection>()).get_index()},
      ego_pose_index_{this->DeclareInputPort(drake::systems::kVectorValued, PoseVector<T>::kSize).get_index()},
      steering_command_index_{
          this->DeclareVectorOutputPort(BasicVector<T>(1), &PurePursuitController::OutputSteeringCommand).get_index()} {
  this->DeclareNumericParameter(PurePursuitParams<T>());
  this->DeclareNumericParameter(SimpleCarParams<T>());
}

template <typename T>
PurePursuitController<T>::~PurePursuitController() {}

template <typename T>
const drake::systems::InputPort<T>& PurePursuitController<T>::lane_input() const {
  return drake::systems::System<T>::get_input_port(lane_index_);
}

template <typename T>
const drake::systems::InputPort<T>& PurePursuitController<T>::ego_pose_input() const {
  return drake::systems::System<T>::get_input_port(ego_pose_index_);
}

template <typename T>
const drake::systems::OutputPort<T>& PurePursuitController<T>::steering_command_output() const {
  return drake::systems::System<T>::get_output_port(steering_command_index_);
}

template <typename T>
void PurePursuitController<T>::OutputSteeringCommand(const drake::systems::Context<T>& context,
                                                     drake::systems::BasicVector<T>* steering_output) const {
  // Obtain the parameters.
  const PurePursuitParams<T>& pp_params =
      this->template GetNumericParameter<PurePursuitParams>(context, kPpParamsIndex);
  const SimpleCarParams<T>& car_params = this->template GetNumericParameter<SimpleCarParams>(context, kCarParamsIndex);

  // Obtain the input/output data structures.
  const LaneDirection* const lane_direction =
      this->template EvalInputValue<LaneDirection>(context, this->lane_input().get_index());
  DRAKE_ASSERT(lane_direction != nullptr);

  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(context, this->ego_pose_input().get_index());
  DRAKE_ASSERT(ego_pose != nullptr);

  CalcSteeringCommand(pp_params, car_params, *lane_direction, *ego_pose, steering_output);
}

template <typename T>
void PurePursuitController<T>::CalcSteeringCommand(const PurePursuitParams<T>& pp_params,
                                                   const SimpleCarParams<T>& car_params,
                                                   const LaneDirection& lane_direction, const PoseVector<T>& ego_pose,
                                                   BasicVector<T>* command) const {
  DRAKE_DEMAND(car_params.IsValid());
  DRAKE_DEMAND(pp_params.IsValid());

  // Compute the steering angle using the pure-pursuit method.  N.B. Assumes
  // zero elevation and superelevation.
  (*command)[0] = PurePursuit<T>::Evaluate(pp_params, car_params, lane_direction, ego_pose);
}

}  // namespace delphyne

// These instantiations must match the API documentation in
// pure_pursuit_controller.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(class ::delphyne::PurePursuitController)
