// Copyright 2017 Toyota Research Institute

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>

#include <drake/lcmt_viewer_draw.hpp>
#include <drake/lcmt_viewer_load_robot.hpp>
#include <drake/systems/rendering/pose_bundle.h>

#include <google/protobuf/message.h>

#include <gtest/gtest.h>

#include <ignition/common/Filesystem.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "delphyne/macros.h"

#define EXPECT_THROW_OF_TYPE(type, statement, msg)                                                           \
  do {                                                                                                       \
    try {                                                                                                    \
      statement;                                                                                             \
    } catch (type const& err) {                                                                              \
      if (std::string(err.what()).find(msg) == std::string::npos) {                                          \
        FAIL() << "Expected error msg containing:" << std::endl                                              \
               << msg << std::endl                                                                           \
               << "Saw error msg:" << std::endl                                                              \
               << err.what() << std::endl;                                                                   \
      }                                                                                                      \
    } catch (std::exception const& err) {                                                                    \
      FAIL() << "Expected " #type << std::endl << "Saw exception type: " << typeid(err).name() << std::endl; \
    }                                                                                                        \
  } while (0)

#define EXPECT_RUNTIME_THROW(st, msg) EXPECT_THROW_OF_TYPE(std::runtime_error, st, msg)
#define EXPECT_ARGUMENT_THROW(st, msg) EXPECT_THROW_OF_TYPE(std::invalid_argument, st, msg)

namespace delphyne {
namespace test {

// Generates a pre-loaded lcmt_viewer_draw message.
//
// @return a loaded lcmt_viewer_draw message.
drake::lcmt_viewer_draw BuildPreloadedDrawMsg();

// Generates a pre-loaded lcmt_viewer_load_robot message.
//
// @return a loaded lcmt_viewer_load_robot message.
drake::lcmt_viewer_load_robot BuildPreloadedLoadRobotMsg();

// Generates a pre-loaded Model_V message.
//
// @return a loaded Model_V message.
ignition::msgs::Model_V BuildPreloadedModelVMsg();

// Generates a pre-loaded pose bundle with model poses
// (@see BuildPreloadedModelVMsg).
//
// @return a drake::systems::rendering::PoseBundle<double> instance.
drake::systems::rendering::PoseBundle<double> BuildPreloadedPoseBundle();

// Asserts that all the array-iterable values from
// lcm_msg match the content of the ign_models object.
//
// @param lcm_msg An lcm viewer draw message with the desired values.
// @param ign_models An ignition messages Model_V with the translated values.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckMsgTranslation(const drake::lcmt_viewer_draw& lcm_msg,
                                               const ignition::msgs::Model_V& ign_models);

// Asserts that all the array-iterable values from lcm_msg match the content of
// the ign_models object.
//
// @param lcm_msg An lcm viewer load robot message with the desired values.
// @param ign_models An ignition messages Model_V with the translated values.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckMsgTranslation(const drake::lcmt_viewer_load_robot& lcm_msg,
                                               const ignition::msgs::Model_V& ign_models);

// Asserts that all the array-iterable values from
// lcm_msg match the content of the scene object.
//
// @param lcm_msg An lcm viewer draw message with the desired values.
// @param ign_models An ignition messages Scene with the translated values.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckMsgTranslation(const drake::lcmt_viewer_draw& lcm_msg,
                                               const ignition::msgs::Scene& scene);

// Asserts that the poses of all the models and links in ign_models
// match the pose in ign_poses that has the same Id.
//
// @param ign_models An ignition messages Model_V with the desired values.
// @param ign_poses An ignition messages Pose_V with the translated values.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckMsgTranslation(const ignition::msgs::Model_V& ign_models,
                                               const ignition::msgs::Pose_V& ign_poses);

// Asserts that the position values found on an array provenient from an
// lcm message are equivalent to those found on the ignition object.
//
// @param lcm_position An array of length 3 with the lcm position values.
// @param ign_position An ignition message of type Vector3d with a position
// value.
// @param tolerance A double containing the translation's tolerance.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckLcmArrayToVector3dEquivalence(const float lcm_position[],
                                                              const ignition::msgs::Vector3d& ign_position,
                                                              double tolerance);

// Asserts that the quaternion values found on an array provenient from an
// lcm message are equivalent to those found on the ignition object.
//
// @param lcm_orientation An array of length 4 with the lcm orientation values.
// @param ign_orientation An ignition messages of type Quaternion.
// @param tolerance A double containing the translation's tolerance.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckLcmArrayToQuaternionEquivalence(const float lcm_orientation[],
                                                                const ignition::msgs::Quaternion& ign_orientation,
                                                                double tolerance);

// Asserts that the color values found on an array provenient from an lcm
// message are equivalent to those found on the ignition object.
//
// @param lcm_color An array of length 4 with the lcm color values.
// @param ign_color An ignition messages of type Color.
// @param tolerance A double containing the translation's tolerance.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckLcmArrayToColorEquivalence(const float lcm_color[],
                                                           const ignition::msgs::Color& ign_color, double tolerance);

// Asserts the equivalence between an lcm geometry type versus an
// ignition messages geometry type.
//
// @param lcm_geometry_type An integer that translates into an lcm
// geometry type.
// @param ign_orientation An ignition geometry type value.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckGeometryTypeEquivalence(int8_t lcm_geometry_type,
                                                        ignition::msgs::Geometry::Type ign_geometry_type);

// Asserts that two protobuf messages are equal.
//
// @param lhs A protobuf messsage.
// @param rhs A second protobuf messsage to compare against.
// @return a google test's AssertionResult.
::testing::AssertionResult CheckProtobufMsgEquality(const google::protobuf::MessageLite& lhs,
                                                    const google::protobuf::MessageLite& rhs);

// A helper class to monitor publications over an ignition
// transport topic.
//
// Example to get an instance of `IgnMonitor`:
// @code{.cpp}
//  auto ign_monitor = MakeSharedIgnMonitor<ignition::msgs::AgentState_V>(TopicName);
// @endcode
// Then all the public methods could be called
//
// @note: This class subscribes a callback method to a topic. Ignition-transport7 doesn't guarantee that the callback
// method is executed while the instance of the class is still alive. To solve that lack of syncronization we introduce
// a std::share_ptr of the IgnMonitor class into the callback method.
// @remarks DO NOT COPY this implementation in production code. It will prevent an
// instance of this class to be properly released.
// TODO(#760) Restore previous implementation of IgnMonitor.
//
// @tparam IGN_TYPE A valid ignition message type.
template <typename IGN_TYPE,
          typename std::enable_if<std::is_base_of<ignition::transport::ProtoMsg, IGN_TYPE>::value, int>::type = 0>
class IgnMonitor {
 public:
  // Creates an IgnMonitor.
  // @tparam IGN_TYPE A valid ignition message type.
  // @param topic_name Valid ignition transport topic name.
  // @returns An instance of IgnMonitor wrapped by std::shared_ptr.
  template <typename IGN_TYPE_F>
  friend std::shared_ptr<IgnMonitor<IGN_TYPE_F>> MakeSharedIgnMonitor(const std::string& topic_name);

  ~IgnMonitor() {}

  // Returns the last received message.
  IGN_TYPE get_last_message() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return last_message_;
  }

  // Returns the count of messages received.
  int get_message_count() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return message_count_;
  }

  // Waits for at least @p message_count messages to have arrived,
  // with a given @p timeout.
  //
  // @param message_count Count of messages to wait for.
  // @param timeout Timeout for the wait.
  // @return Whether the wait succeeded or not (i.e. it timed out).
  template <typename R, typename P>
  bool wait_until(int message_count, std::chrono::duration<R, P> timeout) const {
    return do_until(message_count, timeout, [] {});
  }

  // Waits for at least @p message_count messages to have arrived,
  // with a given @p timeout, while performing the given @p procedure.
  //
  // @param message_count Count of messages to wait for.
  // @param timeout Timeout for the wait.
  // @param procedure Callable to execute within the wait loop
  //                  (upon message arrival)
  // @return Whether the wait succeeded or not (i.e. it timed out).
  template <typename R, typename P>
  bool do_until(int message_count, std::chrono::duration<R, P> timeout, std::function<void()> procedure) const {
    std::unique_lock<std::mutex> guard(mutex_);
    while (message_count_ < message_count) {
      procedure();
      if (cv_.wait_for(guard, timeout) == std::cv_status::timeout) {
        return false;
      }
    }
    return true;
  }

 private:
  // Constructs a IgnMonitor object.
  // @param topic_name Valid ignition transport topic name.
  explicit IgnMonitor(const std::string& topic_name) : topic_name_(topic_name) {}

  // Subscribes to the ignition transport topic.
  // @param A pointer to this instance.
  // @throws std::logic_error When `self_ptr.get()` != `this`.
  void Initialize(std::shared_ptr<IgnMonitor<IGN_TYPE>> self_ptr) {
    DELPHYNE_VALIDATE(self_ptr.get() == this, std::logic_error, "self_ptr must point to `this` instance.");
    // A std::shared_ptr to `this` is passed to a instance variable
    // in order to guarantee that the IgnMonitor instance hasn't been destroyed at
    // the moment that the callback method is executed. See #760.
    self_ptr_ = self_ptr;
    node_.Subscribe(topic_name_, &IgnMonitor::OnTopicMessage, this);
  }

  // Ignition subscriber callback, updating state
  // and notify all threads waiting on this instance.
  //
  // @param message Message received.
  void OnTopicMessage(const IGN_TYPE& message) {
    std::lock_guard<std::mutex> guard(mutex_);
    last_message_ = message;
    message_count_++;
    cv_.notify_all();
  }

  // Topic name.
  const std::string topic_name_;
  // Holds a pointer to this instance.
  // This will cause that the instance of the class won't be deallocated by the SO
  // until the execution of the unit is finished. See #760.
  std::shared_ptr<IgnMonitor<IGN_TYPE>> self_ptr_{nullptr};
  // Received message count.
  int message_count_{0};
  // Last ignition message received.
  IGN_TYPE last_message_{};
  // Mutex to synchronize state read/write operations.
  mutable std::mutex mutex_{};
  // Condition variable for state blocking checks.
  mutable std::condition_variable cv_{};
  // Ignition transport node for subscription.
  // The ignition transport node must be declared after everything its callbacks
  // use.  This ensures that it is constructed after everything it needs is
  // properly setup, and destroyed before everything it needs is destroyed,
  // avoiding a race where the callback for a subscribed topic can be called
  // after the member variables it needs have already been destroyed.
  ignition::transport::Node node_{};
};

template <typename IGN_TYPE_F>
std::shared_ptr<IgnMonitor<IGN_TYPE_F>> MakeSharedIgnMonitor(const std::string& topic_name) {
  auto ign_monitor = std::shared_ptr<IgnMonitor<IGN_TYPE_F>>(new IgnMonitor<IGN_TYPE_F>(topic_name));
  ign_monitor->Initialize(ign_monitor);
  return ign_monitor;
};

// Makes and returns a temporary directory out of a @p template_path,
// whose last six (6) characters MUST be 'XXXXXX'.
// @see mkdtemp
std::string MakeTemporaryDirectory(const std::string& template_path);

// An gtest Test subclass for test fixtures that make use of temporary
// files and/or directories.
class TestWithFiles : public ::testing::Test {
 protected:
  void SetUp() override {
    tmpdir_ = test::MakeTemporaryDirectory("/tmp/XXXXXX");
    DoSetUp();
  }

  virtual void DoSetUp() {}

  void TearDown() override {
    DoTearDown();
    ignition::common::removeAll(tmpdir_);
  }

  virtual void DoTearDown() {}

  const std::string& tmpdir() const { return tmpdir_; }

 private:
  std::string tmpdir_{""};
};

}  // namespace test
}  // namespace delphyne
