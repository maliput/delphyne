// Copyright 2018 Toyota Research Institute

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include <drake/automotive/gen/driving_command.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

#include "delphyne/macros.h"

namespace delphyne {

/// The ConstantVectorSettable is a drake system to set an output port based on
/// some kind of input (the input is expected to be some kind of feedback, but
/// could, in theory, be anyting).  It has a single input for the feedback, and
/// a single output port.
///
/// @tparam T must be a valid Eigin ScalarType
/// @tparam INPUT_VECTOR_TYPE must be a valid BasicVector type that matches
///         the type of output that this input will be connected to.
template <typename T, typename INPUT_VECTOR_TYPE>
class ConstantVectorSettable final : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConstantVectorSettable)

  typedef std::function<T(const drake::systems::BasicVector<T>*, T requested)>
      SettableCallback;
  typedef std::function<T(const drake::systems::BasicVector<T>*)> GetCallback;

  /// Constructs a ConstantVectorSettable given two callbacks; one to get the
  /// current value, and one to set the current value.
  explicit ConstantVectorSettable(GetCallback get_cb, SettableCallback set_cb)
      : get_cb_(get_cb), set_cb_(set_cb) {
    feedback_input_port_index_ =
        this->DeclareVectorInputPort(INPUT_VECTOR_TYPE()).get_index();
    output_port_index_ =
        this->DeclareVectorOutputPort(drake::systems::BasicVector<T>(1),
                                      &ConstantVectorSettable::CalcOutputValue)
            .get_index();
    this->DeclareAbstractState(drake::systems::AbstractValue::Make<T>(T{}));
  }

  ~ConstantVectorSettable() override {}

  std::unique_ptr<drake::systems::AbstractValue> AllocateDefaultAbstractValue()
      const {
    return std::make_unique<drake::systems::Value<T>>(T{});
  }

  std::unique_ptr<drake::systems::AbstractValues> AllocateAbstractState()
      const override {
    std::vector<std::unique_ptr<drake::systems::AbstractValue>> abstract_values(
        kTotalStateValues);
    abstract_values[kStateIndexVal] = AllocateDefaultAbstractValue();
    return std::make_unique<drake::systems::AbstractValues>(
        std::move(abstract_values));
  }

  /// Returns the value stored in @p context.
  T Get(const drake::systems::Context<T>& context) const {
    const drake::systems::AbstractValues* abstract_state =
        &context.get_abstract_state();

    return abstract_state->get_value(kStateIndexVal).GetValue<T>();
  }

  /// Sets a new value that will eventually be synchronized into @p context.
  void Set(T new_val) {
    std::lock_guard<std::mutex> lock(mutex_);
    val_ = new_val;
  }

  const drake::systems::OutputPort<T>& output() const {
    return this->get_output_port(output_port_index_);
  }

  const drake::systems::InputPortDescriptor<T>& feedback_input() const {
    return this->get_input_port(feedback_input_port_index_);
  }

  void SetDefaultState(const drake::systems::Context<T>& context,
                       drake::systems::State<T>* state) const override {
    DELPHYNE_VALIDATE(state != nullptr, std::invalid_argument,
                      "State pointer must not be null");

    const drake::systems::BasicVector<T>* feedback_input =
        this->template EvalVectorInput<drake::systems::BasicVector>(
            context, feedback_input_port_index_);

    T curr_val = get_cb_(feedback_input);

    drake::systems::AbstractValues* abstract_state =
        &state->get_mutable_abstract_state();

    abstract_state->get_mutable_value(kStateIndexVal).GetMutableValue<T>() =
        curr_val;

    std::lock_guard<std::mutex> lock(mutex_);
    val_ = curr_val;
  }

 protected:
  void DoCalcNextUpdateTime(const drake::systems::Context<T>& context,
                            drake::systems::CompositeEventCollection<T>* events,
                            T* time) const override {
    DELPHYNE_VALIDATE(events != nullptr, std::invalid_argument,
                      "Events pointer must not be null");
    DELPHYNE_VALIDATE(time != nullptr, std::invalid_argument,
                      "Time pointer must not be null");

    // An update time calculation is required here to avoid having
    // a NaN value when callling the StepBy method.
    drake::systems::LeafSystem<T>::DoCalcNextUpdateTime(context, events, time);

    const T last_val = Get(context);

    T curr_val;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      curr_val = val_;
    }

    // Has a new speed. Schedule an update event.
    if (last_val != curr_val) {
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
      const std::vector<const drake::systems::UnrestrictedUpdateEvent<T>*>&,
      drake::systems::State<T>* state) const override {
    DELPHYNE_VALIDATE(state != nullptr, std::invalid_argument,
                      "State pointer must not be null");

    T curr_val;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      curr_val = val_;
    }

    drake::systems::AbstractValues* abstract_state =
        &state->get_mutable_abstract_state();

    abstract_state->get_mutable_value(kStateIndexVal).GetMutableValue<T>() =
        curr_val;
  }

  void CalcOutputValue(const drake::systems::Context<T>& context,
                       drake::systems::BasicVector<T>* output) const {
    const drake::systems::BasicVector<T>* feedback_input =
        this->template EvalVectorInput<drake::systems::BasicVector>(
            context, feedback_input_port_index_);

    output->SetAtIndex(0, set_cb_(feedback_input, Get(context)));
  }

 private:
  mutable T val_;

  // The mutex that guards val_.
  mutable std::mutex mutex_;

  // The callbacks that will get called when necessary to call application
  // specific logic to get and set the current value, respectively.
  GetCallback get_cb_;
  SettableCallback set_cb_;

  // The index of the state used to access the internal value.
  static constexpr int kStateIndexVal = 0;

  // The total number of states used by the system.
  static constexpr int kTotalStateValues = 1;

  /********************
   * System Indices
   *******************/
  int feedback_input_port_index_;
  int output_port_index_;
};

}  // namespace delphyne
