// Copyright 2017 Toyota Research Institute

#include <regex>
#include <string>

#include <ignition/common/Console.hh>

#include <ignition/msgs.hh>

#include <ignition/transport.hh>
#include <ignition/transport/log/Playback.hh>

namespace delphyne {

namespace {

// A helper class that bundles ignition-transport log's playback functionality
// with a service-based API to control it.
//
// The advertised services are the following:
// - The `/replayer/pause`: gets triggered by an `ignition::msgs::Empty`
// message, pausing the log playback.
// - The `/replayer/resume`: gets triggered by an `ignition::msgs::Empty`
// message, resuming the log playback.
class Replayer {
 public:
  explicit Replayer(const std::string& logfile)
      : logfile_{logfile}, player_{logfile} {}

  // Advertises the play/resume services and controls the flow of the logfile's
  // playback.
  int Run() {
    // Advertises services.
    if (!node_.Advertise(kPauseServiceName, &Replayer::PauseServiceCallback,
                         this)) {
      ignerr << "Error advertising service [" << kPauseServiceName << "]"
                << std::endl;
      return 1;
    }
    if (!node_.Advertise(kResumeServiceName, &Replayer::ResumeServiceCallback,
                         this)) {
      ignerr << "Error advertising service [" << kResumeServiceName << "]"
             << std::endl;
      return 1;
    }

    // Register all topics to be played-back.
    const int64_t addTopicResult = player_.AddTopic(std::regex(".*"));
    if (addTopicResult == 0) {
      ignerr << "No topics to play back" << std::endl;
      return 1;
    } else if (addTopicResult < 0) {
      ignerr << "Failed to advertise topics: " << addTopicResult << std::endl;
      return 1;
    }

    // Begins playback.
    handle_ = player_.Start();
    if (!handle_) {
      ignerr << "Failed to start playback" << std::endl;
      return 1;
    }

    // Waits until the player stops on its own.
    ignmsg << "Playing all messages in the log file." << std::endl;
    handle_->WaitUntilFinished();
    return 0;
  }

 private:
  // Pause service's handler.
  void PauseServiceCallback(const ignition::msgs::Empty& _req) {
    if (handle_->IsPaused()) {
      ignerr << "Playback was already paused." << std::endl;
    } else {
      handle_->Pause();
      ignmsg << "Playback is now paused." << std::endl;
    }
  }

  // Resume service's handler.
  void ResumeServiceCallback(const ignition::msgs::Empty& _req) {
    if (!handle_->IsPaused()) {
      ignerr << "Playback was already running." << std::endl;
    } else {
      handle_->Resume();
      ignmsg << "Playback is now running." << std::endl;
    }
  }

  // An ignition-transport node to attend services requests.
  ignition::transport::Node node_;

  // Pause service's name.
  static constexpr const char* const kPauseServiceName = "/replayer/pause";

  // Resume service's name.
  static constexpr const char* const kResumeServiceName = "/replayer/resume";

  // Logfile path passed as argument to the constructor.
  const std::string logfile_;

  // Playback object in charge of replaying the logfile.
  ignition::transport::log::Playback player_;

  // player_'s handle pointer.
  ignition::transport::log::PlaybackHandlePtr handle_;
};

int main(int argc, char* argv[]) {
  ignition::common::Console::SetVerbosity(3);

  if (argc < 2) {
    ignerr << "No logfile was provided.\n"
           << "Usage: " << argv[0] << " <path-to-logfile.db>" << std::endl;
    return 1;
  }

  Replayer replayer(argv[1]);

  return replayer.Run();
}

}  // namespace

}  // namespace delphyne

int main(int argc, char* argv[]) { return delphyne::main(argc, argv); }
