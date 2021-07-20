// Copyright 2018 Toyota Research Institute

#include "backend/data_logger.h"

#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/common/SystemPaths.hh>

#include "delphyne/macros.h"
#include "delphyne/utility/package.h"
#include "utility/compression.h"
#include "utility/filesystem.h"

namespace delphyne {

namespace {

// Resolves the given @p filename path into a full path
// suitable for logging purposes.
//
// Absolute paths are passed through unchanged. Relative paths
// resolution applies the following precedence logic:
// - If $DELPHYNE_LOGS_PATH envvar is defined, resolve from
//   $DELPHYNE_LOGS_PATH/logs path.
// - If $HOME envvar is defined, resolve from $HOME/.delphyne/logs
//   path.
// - If everything else fails, resolve from /tmp/delphyne/logs path.
// @returns A full path to @p filename.
std::string ResolveLogPath(const std::string& filename) {
  std::stringstream abs_path;
  // Skip path build if filename is an absolute path.
  if (!IsAbsolutePath(filename)) {
    // Get environmental variables.
    const char* delphyne_logs_path = std::getenv("DELPHYNE_LOGS_PATH");
    if (delphyne_logs_path == NULL) {
      // In case DELPHYNE_LOGS_PATH isn't available.
      igndbg << "Unable to get DELPHYNE_LOGS_PATH "
             << "environment variable." << std::endl;
      const char* home_path = std::getenv("HOME");
      if (home_path == NULL) {
        igndbg << "Unable to get HOME environment variable." << std::endl;
        // Uses /tmp/delphyne as base path.
        abs_path << "/tmp/delphyne";
      } else {
        // Uses ~/.delphyne as base path.
        abs_path << home_path << "/.delphyne";
      }
    } else {
      // In case DELPHYNE_LOGS_PATH is available, uses it.
      abs_path << delphyne_logs_path;
    }
    // Appends the logs directory to the path.
    abs_path << "/logs/";
  }
  // Appends filename to the path.
  abs_path << filename;
  // Returns a string with the generated path.
  return abs_path.str();
}

// Generates a full path to a log file and full path to temporary
// directory to gather log data based on the given @p filename.
// Returned paths are guaranteed to not be present in the filesystem
// at the time of the call. It's on caller behalf to avoid TOCTTOU
// races if necessary.
// @see ResolveLogPath
std::pair<std::string, std::string> GenerateLogPaths(const std::string& filename) {
  // Gets full path based on the given filename.
  const std::string logpath = ResolveLogPath(filename);
  // Splits extension from log path.
  std::string basepath{""};
  std::string extension{""};
  std::tie(basepath, extension) = SplitExtension(logpath);
  if (extension.empty()) {
    extension = "log";
  }
  // Ensures neither file path nor its associated temporary
  // directory exist, adding a number to file base path until
  // such condition holds. Implicit TOCTTOU race, see method
  // documentation.
  int counter = 1;
  std::string unique_logpath = logpath;
  std::string unique_tmppath = basepath;
  while (ignition::common::exists(unique_logpath) || ignition::common::exists(unique_tmppath)) {
    unique_tmppath = basepath + std::to_string(counter++);
    unique_logpath = unique_tmppath + "." + extension;
  }
  return std::make_pair(std::move(unique_logpath), std::move(unique_tmppath));
}

}  // namespace

DataLogger::~DataLogger() {
  if (is_logging()) {
    Stop();
  }
}

void DataLogger::Sync(const ignition::transport::Clock* clock) {
  DELPHYNE_VALIDATE(clock != nullptr, std::runtime_error, "Given clock is null.");
  DELPHYNE_VALIDATE(!is_logging(), std::runtime_error, "Cannot synchronize logs if already running.");
  using ignition::transport::log::RecorderError;
  const RecorderError result = topic_recorder_.Sync(clock);
  DELPHYNE_VALIDATE(result == RecorderError::SUCCESS, std::runtime_error, "Failed to synchronize topic recordings.");
}

void DataLogger::Start(const std::string& filename) {
  DELPHYNE_VALIDATE(!is_logging(), std::runtime_error, "Cannot start logging, already running.");
  std::string logpath = "";
  std::string tmppath = "";
  std::tie(logpath, tmppath) = GenerateLogPaths(filename);
  DELPHYNE_VALIDATE(ignition::common::createDirectories(tmppath), std::runtime_error,
                    "Cannot setup internal temporary directory structure");
  DELPHYNE_VALIDATE(StartTopicRecording(ignition::common::joinPaths(tmppath, "topic.db")), std::runtime_error,
                    "Could not start logging topic messages.");
  package_ = std::make_unique<utility::BundledPackage>(ignition::common::joinPaths(tmppath, "bundle"));
  tmppath_ = std::move(tmppath);
  logpath_ = std::move(logpath);
}

namespace {

// Finds all message instances of the given @p type within @p msg.
//
// @param[in] msg Message to be traversed.
// @param[in] type Message type descriptor to check for.
// @param output Output iterator to collect all pointer references
//               to found messages.
// @tparam OutputIt Output iterator type, taking pointers to
//                  google::protobuf::Message instances.
template <typename OutputIt>
void find_proto_messages(const google::protobuf::Message& msg, const google::protobuf::Descriptor* type,
                         OutputIt output) {
  using google::protobuf::Descriptor;
  using google::protobuf::FieldDescriptor;
  using google::protobuf::Message;
  using google::protobuf::Reflection;
  const Descriptor* descriptor = msg.GetDescriptor();
  if (descriptor->name() == type->name()) {
    *output++ = &msg;
    return;
  }
  const Reflection* reflection = msg.GetReflection();
  const int count = descriptor->field_count();
  for (int i = 0; i < count; ++i) {
    const FieldDescriptor* field_descriptor = descriptor->field(i);
    if (field_descriptor->type() == FieldDescriptor::TYPE_MESSAGE) {
      if (field_descriptor->is_repeated()) {
        for (int i = 0; i < reflection->FieldSize(msg, field_descriptor); ++i) {
          const Message& field_msg = reflection->GetRepeatedMessage(msg, field_descriptor, i);
          find_proto_messages(field_msg, type, output);
        }
      } else {
        const Message& field_msg = reflection->GetMessage(msg, field_descriptor);
        find_proto_messages(field_msg, type, output);
      }
    }
  }
}

// Finds all MsgType instances within @p msg.
// @param[in] msg Message to be traversed.
// @param output Output iterator to collect all pointer references
//               to found messages.
// @see find_proto_messages(const google::protobuf::Message&,
//                          const google::protobuf::Descriptor*,
//                          OutputIt)
// @tparam MsgType google::protobuf::Message subtype to search for.
// @tparam OutputIt Output iterator type, taking pointers to
//                  MsgType instances.
template <typename MsgType, typename OutputIt>
void find_proto_messages(const google::protobuf::Message& msg, OutputIt output) {
  static_assert(std::is_base_of<google::protobuf::Message, MsgType>::value,
                "Given message type is no protobuf message.");

  const MsgType witness_msg;
  const google::protobuf::Descriptor* message_type = witness_msg.GetDescriptor();
  std::vector<const google::protobuf::Message*> messages;
  find_proto_messages(msg, message_type, std::back_inserter(messages));
  std::transform(messages.begin(), messages.end(), output, [](const google::protobuf::Message* msg) {
    const MsgType* typed_msg = dynamic_cast<const MsgType*>(msg);
    DELPHYNE_DEMAND(typed_msg != nullptr);
    return typed_msg;
  });
}

}  // namespace

bool DataLogger::StartTopicRecording(const std::string& logpath) {
  // Creates logging path for all data.
  // Logs every topic. The return value is the number of topics subscribed,
  // or a negative number on error.
  const int64_t num_subscriptions = topic_recorder_.AddTopic(std::regex(".*"));
  if (num_subscriptions < 0) {
    ignerr << "An error occured wheen adding topics to the logger.\n";
    return false;
  }
  // Begin recording all topics.
  using ignition::transport::log::RecorderError;
  const RecorderError result = topic_recorder_.Start(logpath);
  if (RecorderError::SUCCESS != result) {
    ignerr << "Failed to start recording.\n";
    return false;
  }
  return true;
}

void DataLogger::StopTopicRecording() { topic_recorder_.Stop(); }

void DataLogger::CaptureMeshes(const ignition::msgs::Scene& scene_msg) {
  DELPHYNE_VALIDATE(is_logging(), std::runtime_error, "Cannot capture meshes if not logging.");
  // Look up all meshes in the scene.
  std::vector<const ignition::msgs::MeshGeom*> mesh_msgs{};
  find_proto_messages<ignition::msgs::MeshGeom>(scene_msg, std::back_inserter(mesh_msgs));
  // Logs all meshes.
  for (const ignition::msgs::MeshGeom* msg : mesh_msgs) {
    if (msg->filename().empty()) {
      igndbg << "Found mesh with no associated mesh file." << std::endl;
      continue;
    }
    const ignition::common::URI mesh_uri = utility::ToURI(msg->filename());
    if (package_->Resolve(mesh_uri).Valid()) {
      igndbg << "Mesh " << msg->filename() << " already captured." << std::endl;
      continue;
    }
    package_->Add(mesh_uri);
  }
}

void DataLogger::Stop() {
  DELPHYNE_VALIDATE(is_logging(), std::runtime_error, "Cannot stop if not logging.");
  StopTopicRecording();
  package_.reset();
  ZipDirectory(tmppath_, logpath_);
  ignition::common::removeAll(tmppath_);
  tmppath_ = logpath_ = "";
}

}  // namespace delphyne
