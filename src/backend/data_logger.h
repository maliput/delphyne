// Copyright 2018 Toyota Research Institute

#pragma once

#include <memory>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/log/Recorder.hh>

#include "delphyne/utility/package.h"

namespace delphyne {

namespace detail {

/// Compresses the given @p srcpath directory content into the given
/// @p destination_path archive recursively, using the zip format. All
/// @p source_path parent directories are stripped from archive entry
/// names if present.
/// @returns The compression process error code.
/// @throws std::runtime_error if @p source_path is not an existing directory.
/// @throws std::runtime_error if @p destination_path already exists.
int zipDirectory(const std::string& source_path,
                 const std::string& destination_path);

/// Decompresses a zip archive at @p archive_path into the given
/// @p extract_path.
/// @returns The compression process error code.
/// @throws std::runtime_error if @p archive_path is not a file.
/// @throws std::runtime_error if @p extract_path is not a directory.
int unzipDirectory(const std::string& archive_path,
                   const std::string& extract_path);

}  // namespace detail


/// @brief A class for simulation data logging.
/// This class mainly logs all ignition transport topics traffic,
/// but it can also log scene mesh geometries.
///
/// Log files (with '.dz' extension) are glorified zip files
/// containing both an ignition transport topic log file and a bundled
/// package image (see utility::BundledPackage class documentation) with
/// captured meshes, if any.
class DataLogger {
 public:
  DELPHYNE_NO_COPY_NO_MOVE_NO_ASSIGN(DataLogger);

  DataLogger() = default;

  ~DataLogger();

  /// Starts data logging, to be bundled into @p filename (plus the '.dz'
  /// extension).
  ///
  /// Given @p filename can be either an absolute path or a relative
  /// path, which are resolved against the default log location:
  /// $DELPHYNE_LOGS_PATH/logs, $HOME/.delphyne/logs or /tmp/delphyne/logs
  /// in order, based on envvar availability.
  ///
  /// @remarks Any ongoing logging activity will be stopped before
  ///          starting a new one.
  ///
  /// @param[in] filename Log file name.
  /// @returns Whether logging succesfully started or not.
  bool Start(const std::string& filename);

  /// Logs all meshes found in the given @p scene to support
  /// visualization during reproduction.
  ///
  /// @param[in] scene Scene message containing meshes.
  /// @returns true if all meshes were successfully retrieved,
  ///          false otherwise.
  // TODO(hidmic): This API addresses one instance of the more
  // general problem of referenced resources in the main data
  // stream (i.e. topic data). May support generalization.
  bool CaptureMeshes(const ignition::msgs::Scene& scene_msg);

  /// Stops ongoing logging activity (if any) and bundles
  /// all logged data.
  void Stop();

  /// Whether the logger is currently logging or not.
  bool is_logging() const { return !logpath_.empty(); }

  /// Returns the current log archive filename.
  const std::string& logpath() const { return logpath_; }

 private:
  // Starts recording ignition transport topics into the
  // given @p filepath.
  // @param[in] filepath Full path to topic log file.
  // @returns true if recording successfully started,
  //          false otherwise.
  bool StartTopicRecording(const std::string& filepath);

  // Stops any ongoing ignition transport topic recording.
  void StopTopicRecording();

  // @brief Path to temporary directory initially holding
  // archive content.
  std::string tmppath_{""};
  // @brief Path to logging archive.
  std::string logpath_{""};
  // @brief A recorder of ignition transport topics.
  ignition::transport::log::Recorder topic_recorder_;
  // @brief A bundled package to log meshes into.
  std::unique_ptr<utility::BundledPackage> package_{nullptr};
};

}  // namespace delphyne
