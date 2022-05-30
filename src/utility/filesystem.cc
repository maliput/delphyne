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

#include "utility/filesystem.h"

#include <stdexcept>
#include <string>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>

#include "delphyne/macros.h"

namespace delphyne {

bool IsAbsolutePath(const std::string& path) { return path.front() == '/'; }

bool IsValidFilepath(const std::string& path) {
  return !(path.empty() || ignition::common::EndsWith(path, "/") || ignition::common::EndsWith(path, "/.") ||
           ignition::common::EndsWith(path, "/.."));
}

std::string Dirname(const std::string& path) {
  std::string::size_type n = path.rfind('/');
  if (n == std::string::npos) {
    return "";
  }
  return path.substr(0, n + 1);
}

std::pair<std::string, std::string> SplitExtension(const std::string& path) {
  const std::string::size_type n = path.rfind('.');
  if (n != std::string::npos) {
    std::string extension = path.substr(n + 1);
    if (!extension.empty() && extension.find('/') == std::string::npos) {
      return std::make_pair(path.substr(0, n), std::move(extension));
    }
  }
  return std::make_pair(path, "");
}

void WalkDirectory(const std::string& dirpath, const DirectoryWalkFn& walkfn, bool recursive) {
  DELPHYNE_VALIDATE(ignition::common::isDirectory(dirpath), std::runtime_error, dirpath + " is not a directory path.");
  using ignition::common::DirIter;
  for (DirIter it(dirpath); it != DirIter(); ++it) {
    const std::string path = *it;
    walkfn(path);
    if (recursive && ignition::common::isDirectory(path)) {
      WalkDirectory(path, walkfn, recursive);
    }
  }
}

}  // namespace delphyne
