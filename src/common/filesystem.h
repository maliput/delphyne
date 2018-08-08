// Copyright 2018 Toyota Research Institute

#pragma once

#include <functional>
#include <string>
#include <utility>

namespace delphyne {

/// A function to recursively walk a directory.
/// @param[in] path Full path to found file or directory.
/// @see WalkDirectory
using DirectoryWalkFn = std::function<void(const std::string& path)>;

/// Walks the given @p dirpath in a top-down fashion.
/// @param[in] dirpath Path to directory to be walked.
/// @param[in] walkfn Callable to walk found files and directories with.
/// @param[in] recursive Whether to recursively walk inner directories
///                      or not.
/// @throws std::runtime_error if @p dirpath does not refer to an
///                            existing directory.
void WalkDirectory(const std::string& dirpath,
                   const DirectoryWalkFn& walkfn,
                   bool recursive);

/// Checks whether the given @p path is absolute or not.
bool IsAbsolutePath(const std::string& path);

/// Checks whether the given @p path is a valid file path
/// (i.e. no trailing slash, no dot references to the
/// current directory or the parent directory).
bool IsValidFilepath(const std::string& path);

/// Extracts the directory name of the given @p path.
/// @remarks No actual filesystem lookups are performed
///          during directory name extraction.
/// @param[in] path The path to extract a directory name from.
/// @returns The extracted directory name, if any.
std::string Dirname(const std::string& path);

/// Splits the given @p path into a (base path, extension) tuple,
/// extension being the trailing characters preceded by the last
/// dot '.' in the last portion of @p path. As an example,
/// @em /tmp/./../home/file.0.txt would split into
/// @em /tmp/./../home/file.0 and @em txt.
/// @param[in] path The path to split.
/// @returns A (base path, extension) tuple.
std::pair<std::string, std::string> SplitExtension(const std::string& path);


}  // namespace delphyne
