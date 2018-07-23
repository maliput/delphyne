// Copyright 2018 Toyota Research Institute

#pragma once

#include <memory>
#include <string>
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

class SpeedSystem final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpeedSystem)

  /// Default constructor.
  ///
  /// @param[in] topic_name The name of the ignition topic this system will
  /// be subscribed to.
  SpeedSystem() {
    speed_feedback_input_port_index_ =
        this->DeclareVectorInputPort(
                drake::systems::rendering::FrameVelocity<double>())
            .get_index();

    accel_output_port_index_ =
        this->DeclareVectorOutputPort(drake::systems::BasicVector<double>(1),
                                      &SpeedSystem::CalcOutputAcceleration)
            .get_index();

    this->DeclareAbstractState(
        drake::systems::AbstractValue::Make<double>(double{}));
  }

  ~SpeedSystem() override {}

  std::unique_ptr<drake::systems::AbstractValue> AllocateDefaultAbstractValue()
      const {
    return std::make_unique<drake::systems::Value<double>>(double{});
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
  double GetSpeed(const drake::systems::Context<double>& context) const {
    return context.get_abstract_state()
        .get_value(kStateIndexSpeed)
        .GetValue<double>();
  }

  void SetSpeed(double new_speed_mps) {
    DELPHYNE_VALIDATE(new_speed_mps >= 0.0, std::invalid_argument,
                      "Speed must be positive or 0");
    std::lock_guard<std::mutex> lock(speed_mutex_);
    speed_ = new_speed_mps;
  }

  const drake::systems::OutputPort<double>& acceleration_output() const {
    return this->get_output_port(accel_output_port_index_);
  }

  const drake::systems::InputPortDescriptor<double>& feedback_input() const {
    return this->get_input_port(speed_feedback_input_port_index_);
  }

 protected:
  void DoCalcNextUpdateTime(
      const drake::systems::Context<double>& context,
      drake::systems::CompositeEventCollection<double>* events,
      double* time) const override {
    DELPHYNE_VALIDATE(events != nullptr, std::invalid_argument,
                      "Events pointer must not be null");
    DELPHYNE_VALIDATE(time != nullptr, std::invalid_argument,
                      "Time pointer must not be null");

    // An update time calculation is required here to avoid having
    // a NaN value when callling the StepBy method.
    drake::systems::LeafSystem<double>::DoCalcNextUpdateTime(context, events,
                                                             time);

    const double last_speed = GetSpeed(context);

    double curr_speed;
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
          drake::systems::UnrestrictedUpdateEvent<double>>& uu_events =
          events->get_mutable_unrestricted_update_events();

      uu_events.add_event(
          std::make_unique<drake::systems::UnrestrictedUpdateEvent<double>>(
              drake::systems::Event<double>::TriggerType::kTimed));
    }
  }

  void DoCalcUnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      const std::vector<
          const drake::systems::UnrestrictedUpdateEvent<double>*>&,
      drake::systems::State<double>* state) const override {
    DELPHYNE_VALIDATE(state != nullptr, std::invalid_argument,
                      "State pointer must not be null");

    double curr_speed;
    {
      std::lock_guard<std::mutex> lock(speed_mutex_);
      curr_speed = speed_;
    }

    // In the bootstrapping case, speed_ is negative.  We want to grab the
    // current feedback speed from the context, and use that as our "base"
    // speed.
    if (curr_speed < 0.0) {
      const drake::systems::rendering::FrameVelocity<double>* feedback_input =
          this->template EvalVectorInput<
              drake::systems::rendering::FrameVelocity>(
              context, speed_feedback_input_port_index_);
      if (feedback_input == nullptr) {
        // If feedback_input is null, it isn't connected.  In that case, leave
        // the speed negative, since if it does get hooked up we want to
        // initialize it properly.
        return;
      }
      const drake::multibody::SpatialVelocity<double> vel =
          feedback_input->get_velocity();
      curr_speed = vel.translational().norm();
    }

    state->get_mutable_abstract_state()
        .get_mutable_value(kStateIndexSpeed)
        .GetMutableValue<double>() = curr_speed;
  }

  void CalcOutputAcceleration(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const {
    const drake::systems::rendering::FrameVelocity<double>* feedback_input =
        this->template EvalVectorInput<
            drake::systems::rendering::FrameVelocity>(
            context, speed_feedback_input_port_index_);

    if (feedback_input == nullptr) {
      // If the feedback is not connected, we can't do anything, so leave
      // acceleration at 0
      output->SetAtIndex(0, 0.0);
      return;
    }

    const drake::multibody::SpatialVelocity<double> vel =
        feedback_input->get_velocity();

    // Let's calculate the magnitude of the vector to get an estimate
    // of our forward speed.
    double magnitude = vel.translational().norm();

    double input_speed = GetSpeed(context);

    if (input_speed > magnitude) {
      output->SetAtIndex(0, 1.0);
    } else if (input_speed < magnitude) {
      output->SetAtIndex(0, -1.0);
    } else {
      output->SetAtIndex(0, 0.0);
    }
  }

 private:
  mutable double speed_{-1.0};

  // The mutex that guards speed_.
  mutable std::mutex speed_mutex_;

  // The index of the state used to access the the speed.
  static constexpr int kStateIndexSpeed = 0;

  // The total number of states used by the system.
  static constexpr int kTotalStateValues = 1;

  /********************
   * System Indices
   *******************/
  int speed_feedback_input_port_index_;
  int accel_output_port_index_;
};

}  // namespace delphyne
