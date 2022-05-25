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
void WalkDirectory(const std::string& dirpath, const DirectoryWalkFn& walkfn, bool recursive);

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
