// Copyright 2017 Toyota Research Institute

#include <cstdint>
#include <iostream>
#include <regex>
#include <ignition/common/Console.hh>
#include <ignition/transport/log/Playback.hh>

namespace backend {

int main(int argc, char* argv[]) {
  ignition::common::Console::SetVerbosity(3);
  if (argc != 2) {
    ignerr << "No logfile was provided.\n"
           << "Usage: " << argv[0] << " <path-to-logfile.db>" << std::endl;
    return 1;
  }

  // Creates a Playback with the logfile passed via argument.
  ignition::transport::log::Playback player(argv[1]);

  // Playbacks all topics.
  const int64_t addTopicResult = player.AddTopic(std::regex(".*"));
  if (addTopicResult == 0) {
    ignerr << "No topics to play back" << std::endl;
    return 1;
  } else if (addTopicResult < 0) {
    ignerr << "Failed to advertise topics: " << addTopicResult << std::endl;
    return 1;
  }

  // Begins playback.
  const auto handle = player.Start();
  if (!handle) {
    ignerr << "Failed to start playback" << std::endl;
    return 1;
  }

  // Waits until the player stops on its own.
  ignmsg << "Playing all messages in the log file." << std::endl;
  handle->WaitUntilFinished();

  return 0;
}

}  // namespace backend

int main(int argc, char* argv[]) { return backend::main(argc, argv); }
