// Copyright 2018 Toyota Research Institute

#pragma once

#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/framework/leaf_system.h>

#include "delphyne/macros.h"

namespace delphyne {

// TODO(clalancette): We should allow VectorSource to be an N-vector,
// rather than just the 1-vector it is now.  @stonier's idea here was
// to take a map of strings to indices in the constructor.  The Set()
// method would then take a string and a value, look up the index based
// on the string, and set it to that value.

/// The VectorSource is a drake system to continually set an output
/// port to a fixed value until the user decides to change it via the `Set`
/// API.  At that point, the value will be (safely) updated to the new value,
/// and the ConstantVectorSettable will continue outputting that value until
/// the user asks for a new one.  This differs from drake's ConstantVectorSource
/// in that it is thread-safe with respect to changes to the ConstantVector.
///
/// @tparam T must be a valid Eigen ScalarType
template <typename T>
class VectorSource final : public drake::systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VectorSource)

  explicit VectorSource(T defaultval) {
    output_port_index_ =
        this->DeclareVectorOutputPort(drake::systems::BasicVector<T>(1),
                                      &VectorSource::CalcOutputValue)
            .get_index();
    this->DeclareAbstractState(
        drake::AbstractValue::Make<T>(T{defaultval}));
    val_ = defaultval;
  }

  ~VectorSource() override {}

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
    const drake::systems::AbstractValues* abstract_state =
        &context.get_abstract_state();

    output->SetAtIndex(0,
                       abstract_state->get_value(kStateIndexVal).GetValue<T>());
  }

 private:
  mutable T val_;

  // The mutex that guards val_.
  mutable std::mutex mutex_;

  // The index of the state used to access the internal value.
  static constexpr int kStateIndexVal = 0;

  // The total number of states used by the system.
  static constexpr int kTotalStateValues = 1;

  /********************
   * System Indices
   *******************/
  int output_port_index_;
};

}  // namespace delphyne
