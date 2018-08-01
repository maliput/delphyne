// Copyright 2018 Toyota Research Institute

#include "common/filesystem.h"

#include <stdexcept>
#include <string>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>

#include "delphyne/macros.h"

namespace delphyne {

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

void WalkDirectory(const std::string& dirpath,
                   const DirectoryWalkFn& walkfn,
                   bool recursive) {
  DELPHYNE_VALIDATE(ignition::common::isDirectory(dirpath),
                    std::runtime_error, dirpath +
                    " is not a directory path.");
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
