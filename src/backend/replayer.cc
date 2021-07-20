// Copyright 2017 Toyota Research Institute

#include <chrono>
#include <regex>
#include <string>
#include <thread>

#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <ignition/transport/log/Batch.hh>
#include <ignition/transport/log/Log.hh>
#include <ignition/transport/log/Message.hh>
#include <ignition/transport/log/Playback.hh>
#include <ignition/transport/log/QueryOptions.hh>

#include "delphyne/macros.h"
#include "delphyne/protobuf/playback_status.pb.h"
#include "delphyne/protobuf/scene_request.pb.h"

namespace delphyne {
namespace {

// Populates @p dst time based on given @p src timestamp,
// converting one representation to the other.
void ChronoToIgnTime(const std::chrono::nanoseconds& src, ignition::msgs::Time* dst) {
  DELPHYNE_DEMAND(dst != nullptr);
  std::chrono::seconds src_in_seconds = std::chrono::duration_cast<std::chrono::seconds>(src);
  dst->set_sec(std::floor(src_in_seconds.count()));
  dst->set_nsec((src - src_in_seconds).count());
}

// A helper class that bundles ignition-transport log's playback functionality
// with a service-based API to control it.
//
// The advertised topics are:
// - */replayer/status*: an ignition::msgs::PlaybackStatus message published at
//                       a fixed rate to track playback state.
// The advertised services are:
// - */replayer/pause*: expects an ignition::msgs::Empty request message,
//                      pausing a running log playback.
// - */replayer/resume*: expects an ignition::msgs::Empty request message,
//                       resuming a paused log playback.
// - */replayer/seek*: expects an ignition::msgs::Duration request message,
//                     establishing the playback time at the given time offset
//                     with respect to the playback start time
// - */replayer/step*: expects an ignition::msgs::Duration request message,
//                     stepping a log playback by a given time.
// - */get_scene*: expects an empty request message sent by the visualizer to
//                 retrieve the whole simulation scene. In this case, the
//                 first scene message found in the log is returned.
class Replayer {
 public:
  // Constructs a replayer for the given @p logfile.
  // @param[in] logfle Path to the record log file.
  // @throws std::runtime_error if @p logfile is not a valid log file.
  explicit Replayer(const std::string& logfile) : player_{logfile} {
    DELPHYNE_VALIDATE(log_.Open(logfile, std::ios::in), std::runtime_error,
                      "Cannot open provided " + logfile + " log file.");
  }

  // Runs playback, advertising topics and services for status and control.
  int Run() {
    // Register all topics to be played-back.
    const int64_t topics_add_result = player_.AddTopic(std::regex(".*"));
    if (topics_add_result == 0) {
      ignerr << "No topics to play back" << std::endl;
      return 1;
    }
    if (topics_add_result < 0) {
      ignerr << "Failed to advertise topics: " << topics_add_result << std::endl;
      return 1;
    }
    // Begins playback.
    handle_ = player_.Start();
    if (!handle_) {
      ignerr << "Failed to start playback" << std::endl;
      return 1;
    }
    // Sets up all playback-specific topics and services.
    if (!SetupSceneServices()) {
      ignerr << "Cannot provide scene services." << std::endl;
      return 1;
    }
    if (!SetupPlaybackServices()) {
      ignerr << "Cannot provide playback services." << std::endl;
      return 1;
    }

    using ignition::transport::Node;
    Node::Publisher status_pub = SetupPlaybackStatusPublication();
    if (!status_pub) {
      ignerr << "Cannot provide playback status." << std::endl;
      return 1;
    }
    // Fills up playback status message with immutable fields first.
    ignition::msgs::PlaybackStatus msg;
    ChronoToIgnTime(handle_->StartTime(), msg.mutable_start_time());
    ChronoToIgnTime(handle_->EndTime(), msg.mutable_end_time());
    // Loops until playback is over, publishing status periodically
    // at the given rate.
    const std::chrono::milliseconds kStatusUpdatePeriod{15};  // Roughly 60Hz.
    std::chrono::time_point<std::chrono::steady_clock> next_status_update_time =
        (std::chrono::steady_clock::now() + kStatusUpdatePeriod);
    while (!handle_->Finished()) {
      // Updates playback status and publishes it.
      ChronoToIgnTime(handle_->CurrentTime(), msg.mutable_current_time());
      msg.set_paused(handle_->IsPaused());
      status_pub.Publish(msg);
      // Waits until the given time point.
      std::this_thread::sleep_until(next_status_update_time);
      next_status_update_time = (std::chrono::steady_clock::now() + kStatusUpdatePeriod);
    }
    return 0;
  }

 private:
  // Sets up playback status publication for the visualizer to use.
  ignition::transport::Node::Publisher SetupPlaybackStatusPublication() {
    constexpr const char* const kStatusTopicName = "/replayer/status";
    using ignition::transport::Node;
    Node::Publisher pub = node_.Advertise<ignition::msgs::PlaybackStatus>(kStatusTopicName);
    if (!pub) {
      ignerr << "Error advertising topic [" << kStatusTopicName << "]" << std::endl;
    }
    return pub;
  }

  // Sets up playback related services for the visualizer to use.
  bool SetupPlaybackServices() {
    constexpr const char* const kPauseServiceName = "/replayer/pause";
    constexpr const char* const kResumeServiceName = "/replayer/resume";
    constexpr const char* const kSeekServiceName = "/replayer/seek";
    constexpr const char* const kStepServiceName = "/replayer/step";

    // Advertises pause and resume services.
    if (!node_.Advertise(kPauseServiceName, &Replayer::OnPauseRequestCallback, this)) {
      ignerr << "Error advertising service [" << kPauseServiceName << "]" << std::endl;
      return false;
    }
    if (!node_.Advertise(kResumeServiceName, &Replayer::OnResumeRequestCallback, this)) {
      ignerr << "Error advertising service [" << kResumeServiceName << "]" << std::endl;
      return false;
    }
    if (!node_.Advertise(kStepServiceName, &Replayer::OnStepRequestCallback, this)) {
      ignerr << "Error advertising service [" << kStepServiceName << "]" << std::endl;
      return false;
    }
    if (!node_.Advertise(kSeekServiceName, &Replayer::OnSeekRequestCallback, this)) {
      ignerr << "Error advertising service [" << kSeekServiceName << "]" << std::endl;
      return false;
    }
    return true;
  }

  // Pause service's handler.
  void OnPauseRequestCallback(const ignition::msgs::Empty&) {
    if (!handle_->IsPaused()) {
      handle_->Pause();
      ignmsg << "Playback is now paused." << std::endl;
    } else {
      igndbg << "Playback was already paused." << std::endl;
    }
  }

  // Resume service's handler.
  void OnResumeRequestCallback(const ignition::msgs::Empty&) {
    if (handle_->IsPaused()) {
      handle_->Resume();
      ignmsg << "Playback is now running." << std::endl;
    } else {
      igndbg << "Playback was already running." << std::endl;
    }
  }

  // Step service's handler.
  void OnStepRequestCallback(const ignition::msgs::Duration& step_duration) {
    if (!handle_->IsPaused()) {
      ignerr << "Playback must be paused to step." << std::endl;
    } else {
      const std::chrono::nanoseconds total_nanos{
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds(step_duration.sec())) +
          std::chrono::nanoseconds(step_duration.nsec())};
      const std::chrono::milliseconds total_millis{std::chrono::duration_cast<std::chrono::milliseconds>(total_nanos)};
      igndbg << "Stepping playback for " << total_millis.count() << " milliseconds." << std::endl;
      handle_->Step(total_nanos);
    }
  }

  // Seek service's handler.
  void OnSeekRequestCallback(const ignition::msgs::Duration& seek_duration) {
    using seconds = std::chrono::duration<double>;
    const std::chrono::nanoseconds total_nanos{std::chrono::seconds(seek_duration.sec()) +
                                               std::chrono::nanoseconds(seek_duration.nsec())};
    const seconds total_secs{total_nanos};
    ignmsg << "Establishing playback time to " << total_secs.count() << " seconds." << std::endl;
    handle_->Seek(total_nanos);
  }

  // Sets up scene related services for the visualizer to use.
  bool SetupSceneServices() {
    using ignition::transport::log::Batch;
    using ignition::transport::log::Message;
    using ignition::transport::log::TopicList;
    constexpr const char* const kSceneTopicName = "/scene";
    constexpr const char* const kSceneRequestServiceName = "/get_scene";
    // Retrieves first Scene message from the log.
    Batch scene_messages_batch = log_.QueryMessages(TopicList(std::set<std::string>{kSceneTopicName}));
    if (scene_messages_batch.begin() == scene_messages_batch.end()) {
      ignwarn << "No scene messages found in the log." << std::endl;
      return false;
    }
    Batch::iterator it = scene_messages_batch.begin();
    if (!first_scene_message_.ParseFromString(it->Data())) {
      ignwarn << "Messages in " << kSceneTopicName << " are not "
              << "scene messages." << std::endl;
      return false;
    }
    // Advertises the scene request service.
    if (!node_.Advertise(kSceneRequestServiceName, &Replayer::OnSceneRequestCallback, this)) {
      ignwarn << "Error advertising service "
              << "[" << kSceneRequestServiceName << "]" << std::endl;
      return false;
    }
    return true;
  }

  // Callback for the scene request service.
  bool OnSceneRequestCallback(ignition::msgs::Scene& response) {
    // Replies to the caller with the first Scene message found in logs.
    response = first_scene_message_;
    return true;
  }

  // First scene message found in the log, if any.
  ignition::msgs::Scene first_scene_message_;

  // An ignition-transport node to attend services requests.
  ignition::transport::Node node_;

  // Log object wrapping the given logfile.
  ignition::transport::log::Log log_;

  // Playback object in charge of replaying the logs.
  ignition::transport::log::Playback player_;

  // Player_'s object handle.
  ignition::transport::log::PlaybackHandlePtr handle_;
};

int main(int argc, char** argv) {
  ignition::common::Console::SetVerbosity(3);

  if (argc < 2) {
    ignerr << "No logfile was provided.\n"
           << "Usage: " << argv[0] << " <path-to-logfile>" << std::endl;
    return 1;
  }

  Replayer replayer(argv[1]);

  return replayer.Run();
}

}  // namespace
}  // namespace delphyne

int main(int argc, char* argv[]) { return delphyne::main(argc, argv); }
