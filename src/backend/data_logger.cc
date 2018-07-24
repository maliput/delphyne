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

#include "delphyne/macros.h"
#include "delphyne/utility/package.h"

namespace delphyne {

namespace {

std::pair<std::string, std::string> SplitBasename(const std::string& path) {
  const std::string::size_type n = path.rfind('/');
  if (n == std::string::npos) {
    return std::make_pair("", path);
  }
  return std::make_pair(path.substr(0, n + 1), path.substr(n + 1));
}

std::pair<std::string, std::string> SplitExtension(const std::string& path) {
  const std::string::size_type n = path.rfind('.');
  if (n != std::string::npos) {
    std::string extension = path.substr(n);
    if (!ignition::common::EndsWith(extension, ".") &&
        extension.find('/') == std::string::npos) {
      return std::make_pair(path.substr(0, n), std::move(extension));
    }
  }
  return std::make_pair(path, "");
}


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
  if (filename.front() != '/') {
    // Get environmental variables.
    const char* delphyne_logs_path =
        std::getenv("DELPHYNE_LOGS_PATH");
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
// at the time of the call.
// @see ResolveLogPath
std::pair<std::string, std::string>
GenerateLogPaths(const std::string& filename) {
  // Gets full path based on the given filename.
  const std::string logpath = ResolveLogPath(filename);
  // Splits extension from log path.
  std::string basepath; std::string extension;
  std::tie(basepath, extension) = SplitExtension(logpath);
  if (extension.empty()) extension = ".log";
  // Ensures neither file path nor its associated temporary
  // directory exist, adding a number to file base path until
  // such condition holds.
  int counter = 1;
  std::string unique_logpath = logpath;
  std::string unique_tmppath = basepath;
  while (ignition::common::exists(unique_logpath) ||
         ignition::common::exists(unique_tmppath)) {
    unique_tmppath = basepath + std::to_string(counter++);
    unique_logpath = unique_tmppath + extension;
  }
  return std::make_pair(std::move(unique_logpath),
                        std::move(unique_tmppath));
}

}  // namespace

DataLogger::~DataLogger() {
  if (is_logging()) {
    Stop();
  }
}

bool DataLogger::Start(const std::string& filename) {
  DELPHYNE_VALIDATE(!is_logging(), std::runtime_error,
                    "Cannot start logging, already running.");
  std::string logpath = "";
  std::string tmppath = "";
  std::tie(logpath, tmppath) = GenerateLogPaths(filename);
  if (ignition::common::createDirectories(tmppath)) {
    if (StartTopicRecording(ignition::common::joinPaths(tmppath, "topic.db"))) {
      package_ = std::make_unique<utility::BundledPackage>(
          ignition::common::joinPaths(tmppath, "bundle"));
      tmppath_ = std::move(tmppath);
      logpath_ = std::move(logpath);
      return true;
    }
    ignition::common::removeAll(tmppath);
  }
  return false;
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
void find_proto_messages(const google::protobuf::Message& msg,
                         const google::protobuf::Descriptor* type,
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
          const Message& field_msg =
              reflection->GetRepeatedMessage(msg, field_descriptor, i);
          find_proto_messages(field_msg, type, output);
        }
      } else {
        const Message& field_msg =
          reflection->GetMessage(msg, field_descriptor);
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
void find_proto_messages(const google::protobuf::Message& msg,
                         OutputIt output) {
  static_assert(std::is_base_of<google::protobuf::Message, MsgType>::value,
                "Given message type is no protobuf message.");

  const MsgType witness_msg;
  const google::protobuf::Descriptor* message_type =
      witness_msg.GetDescriptor();
  std::vector<const google::protobuf::Message*> messages;
  find_proto_messages(msg, message_type, std::back_inserter(messages));
  std::transform(messages.begin(), messages.end(),
                 output, [](const google::protobuf::Message* msg) {
                   const MsgType* typed_msg =
                       dynamic_cast<const MsgType*>(msg);
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

void DataLogger::StopTopicRecording() {
  topic_recorder_.Stop();
}

bool DataLogger::CaptureMeshes(const ignition::msgs::Scene& scene_msg) {
  DELPHYNE_VALIDATE(is_logging(), std::runtime_error,
                    "Cannot capture meshes if not logging.");
  // Look up all meshes in the scene.
  std::vector<const ignition::msgs::MeshGeom*> mesh_msgs{};
  find_proto_messages<ignition::msgs::MeshGeom>(
      scene_msg, std::back_inserter(mesh_msgs));
  // Logs all meshes.
  utility::PackageManager* package_manager =
      utility::PackageManager::Instance();
  const utility::Package& package_in_use =
      package_manager->package_in_use();
  for (const ignition::msgs::MeshGeom* msg : mesh_msgs) {
    if (msg->has_filename()) {
      const ignition::common::URI mesh_uri = utility::ToURI(msg->filename());
      if (!package_->Exists(mesh_uri)) {
        if (package_in_use.Exists(mesh_uri)) {
          const std::string mesh_path = package_in_use.Find(mesh_uri);
          if (!package_->Add(mesh_uri, mesh_path)) return false;
        } else {
          ignwarn << "Could not resolve " << mesh_uri.Str() << std::endl;
        }
      }
    }
  }
  return true;
}

namespace detail {

// TODO(hidmic): Find a proper cross-platform mechanism to zip a
//               directory recursively.
int zipDirectory(const std::string& source_path,
                 const std::string& destination_path) {
  DELPHYNE_VALIDATE(
      ignition::common::isDirectory(source_path), std::runtime_error,
      "Given source path is not an existing directory.");
  DELPHYNE_VALIDATE(
      !ignition::common::exists(destination_path), std::runtime_error,
      "Given destination path already exists.");
  // Zip `source_path` contents recursively and output to `destination_path`.
  std::stringstream cmdline_builder;
  cmdline_builder << "cd " << source_path << ";"
                  << "zip -r " << destination_path
                  << " * > /dev/null";
  const std::string cmdline = cmdline_builder.str();
  return std::system(cmdline.c_str());
}

// TODO(hidmic): Find a proper cross-platform mechanism to unzip.
int unzipDirectory(const std::string& archive_path,
                   const std::string& extract_path) {
  DELPHYNE_VALIDATE(ignition::common::isFile(archive_path), std::runtime_error,
                    "Given source path is not an existing directory.");
  DELPHYNE_VALIDATE(
      ignition::common::isDirectory(extract_path), std::runtime_error,
      "Given extraction path must be an existing directory.");
  std::stringstream cmdline_builder;
  // Unzip `archive_path` into `extract_path`, quietly (-q) and
  // overwriting any files without prompting.
  cmdline_builder << "unzip -o -q " << archive_path
                  << " -d " << extract_path
                  << " > /dev/null";
  const std::string cmdline = cmdline_builder.str();
  return std::system(cmdline.c_str());
}

}  // namespace detail

void DataLogger::Stop() {
  DELPHYNE_VALIDATE(is_logging(), std::runtime_error,
                    "Cannot stop if not logging.");
  StopTopicRecording();
  package_.reset();
  if (detail::zipDirectory(tmppath_, logpath_) != 0) {
    ignerr << "Failed to compress log data." << std::endl;
  }
  ignition::common::removeAll(tmppath_);
  tmppath_ = logpath_ = "";
}

}  // namespace delphyne
