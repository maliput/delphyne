// Copyright 2018 Toyota Research Institute

#pragma once

#include <string>
#include <utility>

namespace delphyne {

/// Compresses the given @p srcpath directory content into the given
/// @p destination_path archive recursively, using the zip format. All
/// @p source_path parent directories are stripped from archive entry
/// names if present.
/// @throws std::runtime_error if @p source_path is not an existing directory.
/// @throws std::runtime_error if @p destination_path already exists.
/// @throws std::runtime_error if the zipping process fails at any point.
void ZipDirectory(const std::string& source_path, const std::string& destination_path);

/// Decompresses a zip archive at @p archive_path into the given
/// @p extract_path.
/// @throws std::runtime_error if @p archive_path is not a file.
/// @throws std::runtime_error if @p extract_path is not a directory.
/// @throws std::runtime_error if the unzipping process fails at any point.
void UnzipDirectory(const std::string& archive_path, const std::string& extract_path);

}  // namespace delphyne
