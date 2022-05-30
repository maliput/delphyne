// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2018-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <memory>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/log/Recorder.hh>

#include "delphyne/utility/package.h"

namespace delphyne {

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

  /// Synchronizes data logging with the given @p clock.
  /// @throws std::runtime_error if clock is nullptr.
  /// @throws std::runtime_error if the logger is running already
  ///                            (i.e. is_logging() is true).
  /// @throws std::runtime_error if it fails to synchronize with the
  ///                            given @p clock.
  void Sync(const ignition::transport::Clock* clock);

  /// Starts data logging, to be bundled into @p filename (plus the '.dz'
  /// extension).
  ///
  /// Given @p filename can be either an absolute path or a relative
  /// path, which are resolved against the default log location:
  /// $DELPHYNE_LOGS_PATH/logs, $HOME/.delphyne/logs or /tmp/delphyne/logs
  /// in order, based on envvar availability.
  ///
  /// @param[in] filename Log file name.
  /// @throws std::runtime_error if the logger is running already
  ///                            (i.e. is_logging() is true).
  /// @throws std::runtime_error if it fails to setup any associated
  ///                            files in the filesystem.
  /// @throws std::runtime_error if it cannot start logging topic messages.
  void Start(const std::string& filename);

  /// Logs all meshes found in the given @p scene_msg to support later
  /// visualization during reproduction.
  ///
  /// @param[in] scene_msg Scene message containing meshes.
  /// @throws std::runtime_error if the logger is not running already
  ///                            (i.e. is_logging() is false).
  /// @throws std::runtime_error if it fails to capture a mesh
  ///                            (@see utility::BundledPackage::Add).
  // TODO(hidmic): This API addresses one instance of the more
  // general problem of referenced resources in the main data
  // stream (i.e. topic data). May support generalization.
  void CaptureMeshes(const ignition::msgs::Scene& scene_msg);

  /// Stops ongoing logging activity and bundles all logged data.
  /// @throws std::runtime_error if the logger is not running already
  ///                            (i.e. is_logging() is false).
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
  // @brief A recorder of ignition transport topics, synchronized
  // with the simulation time.
  ignition::transport::log::Recorder topic_recorder_;
  // @brief A bundled package to log meshes into.
  std::unique_ptr<utility::BundledPackage> package_{nullptr};
};

}  // namespace delphyne
