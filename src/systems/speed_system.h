// Copyright 2018 Toyota Research Institute

#pragma once

#include <cmath>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include <drake/automotive/gen/driving_command.h>
#include <drake/lcmt_viewer_draw.hpp>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "delphyne/macros.h"
#include "delphyne/protobuf/automotive_driving_command.pb.h"

namespace delphyne {

/// The SpeedSystem implements a very simple speed controller, taking as an
/// input the current frame velocity (from an InputPort) and the desired speed
/// (set as an abstract state value of this class), and producing an
/// acceleration on an OutputPort to reach that speed.
template<typename T>
class SpeedSystem final : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpeedSystem)

  /// Default constructor.
  SpeedSystem() {
    speed_feedback_input_port_index_ =
        this->DeclareVectorInputPort(
                drake::systems::rendering::FrameVelocity<T>())
            .get_index();

    accel_output_port_index_ =
        this->DeclareVectorOutputPort(drake::systems::BasicVector<T>(1),
                                      &SpeedSystem::CalcOutputAcceleration)
            .get_index();

    this->DeclareAbstractState(
        drake::systems::AbstractValue::Make<T>(T{}));
  }

  ~SpeedSystem() override {}

  std::unique_ptr<drake::systems::AbstractValue> AllocateDefaultAbstractValue()
      const {
    return std::make_unique<drake::systems::Value<T>>(T{});
  }

  std::unique_ptr<drake::systems::AbstractValues> AllocateAbstractState()
      const override {
    std::vector<std::unique_ptr<drake::systems::AbstractValue>> abstract_values(
        kTotalStateValues);
    abstract_values[kStateIndexSpeed] = AllocateDefaultAbstractValue();
    return std::make_unique<drake::systems::AbstractValues>(
        std::move(abstract_values));
  }

  /// Returns the speed stored in @p context.
  T GetSpeed(const drake::systems::Context<T>& context) const {
    const drake::systems::AbstractValues* abstract_state =
        &context.get_abstract_state();

    return abstract_state->get_value(kStateIndexSpeed).GetValue<T>();
  }

  void SetSpeed(T new_speed_mps) {
    DELPHYNE_VALIDATE(new_speed_mps >= 0.0, std::invalid_argument,
                      "Speed must be positive or 0");
    std::lock_guard<std::mutex> lock(speed_mutex_);
    speed_ = new_speed_mps;
  }

  const drake::systems::OutputPort<T>& acceleration_output() const {
    return this->get_output_port(accel_output_port_index_);
  }

  const drake::systems::InputPortDescriptor<T>& feedback_input() const {
    return this->get_input_port(speed_feedback_input_port_index_);
  }

 protected:
  void DoCalcNextUpdateTime(
      const drake::systems::Context<T>& context,
      drake::systems::CompositeEventCollection<T>* events,
      T* time) const override {
    DELPHYNE_VALIDATE(events != nullptr, std::invalid_argument,
                      "Events pointer must not be null");
    DELPHYNE_VALIDATE(time != nullptr, std::invalid_argument,
                      "Time pointer must not be null");

    // An update time calculation is required here to avoid having
    // a NaN value when calling the StepBy method.
    drake::systems::LeafSystem<T>::DoCalcNextUpdateTime(context, events,
                                                        time);

    const T last_speed = GetSpeed(context);

    T curr_speed;
    {
      std::lock_guard<std::mutex> lock(speed_mutex_);
      curr_speed = speed_;
    }

    // Has a new speed. Schedule an update event.
    if (last_speed != curr_speed) {
      // From Drake LCM implementation:
      // TODO(siyuan): should be context.get_time() once #5725 is resolved.
      *time = context.get_time() + 0.0001;

      drake::systems::EventCollection<
          drake::systems::UnrestrictedUpdateEvent<T>>& uu_events =
          events->get_mutable_unrestricted_update_events();

      uu_events.add_event(
          std::make_unique<drake::systems::UnrestrictedUpdateEvent<T>>(
              drake::systems::Event<T>::TriggerType::kTimed));
    }
  }

  void DoCalcUnrestrictedUpdate(
      const drake::systems::Context<T>& context,
      const std::vector<
          const drake::systems::UnrestrictedUpdateEvent<T>*>&,
      drake::systems::State<T>* state) const override {
    DELPHYNE_VALIDATE(state != nullptr, std::invalid_argument,
                      "State pointer must not be null");

    T curr_speed;
    {
      std::lock_guard<std::mutex> lock(speed_mutex_);
      curr_speed = speed_;
    }

    drake::systems::AbstractValues* abstract_state =
      &state->get_mutable_abstract_state();

    abstract_state->get_mutable_value(kStateIndexSpeed).GetMutableValue<T>() =
        curr_speed;
  }

  void CalcOutputAcceleration(
      const drake::systems::Context<T>& context,
      drake::systems::BasicVector<T>* output) const {
    const drake::systems::rendering::FrameVelocity<T>* feedback_input =
        this->template EvalVectorInput<
            drake::systems::rendering::FrameVelocity>(
            context, speed_feedback_input_port_index_);

    if (feedback_input == nullptr) {
      // If the feedback is not connected, we can't do anything, so leave
      // acceleration at 0
      output->SetAtIndex(0, 0.0);
      return;
    }

    T input_speed = GetSpeed(context);

    if (input_speed < 0.0) {
      // If the input_speed is negative, that means we shouldn't do anything in
      // this controller so we just set the acceleration to 0.
      output->SetAtIndex(0, 0.0);
      return;
    }

    const drake::multibody::SpatialVelocity<T> vel =
        feedback_input->get_velocity();

    // Let's calculate the magnitude of the vector to get an estimate
    // of our forward speed.
    T magnitude = vel.translational().norm();

    if (input_speed > magnitude) {
      output->SetAtIndex(0, kAccelerationSetPoint);
    } else if (input_speed < magnitude) {
      output->SetAtIndex(0, -kAccelerationSetPoint);
    } else {
      output->SetAtIndex(0, 0.0);
    }
  }

 private:
  mutable T speed_{-1.0};

  // The mutex that guards speed_.
  mutable std::mutex speed_mutex_;

  // The index of the state used to access the the speed.
  static constexpr int kStateIndexSpeed = 0;

  // The total number of states used by the system.
  static constexpr int kTotalStateValues = 1;

  // The amount of acceleration that will be applied if the acceleration needs
  // to be changed.
  static constexpr T kAccelerationSetPoint = 10.0;

  /********************
   * System Indices
   *******************/
  int speed_feedback_input_port_index_;
  int accel_output_port_index_;
};

}  // namespace delphyne
