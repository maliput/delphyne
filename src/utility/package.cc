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

#include "delphyne/utility/package.h"

#include <cstdlib>
#include <list>
#include <memory>
#include <string>
#include <utility>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/URI.hh>

#include "delphyne/macros.h"
#include "utility/filesystem.h"

namespace delphyne {
namespace utility {

ignition::common::URI ToURI(const std::string& uri_or_path) {
  if (ignition::common::StartsWith(uri_or_path, "/")) {
    return ignition::common::URI("file://" + uri_or_path);
  }
  if (uri_or_path.find("://") == std::string::npos) {
    return ignition::common::URI("package://" + uri_or_path);
  }
  return ignition::common::URI(uri_or_path);
}

SystemPackage::SystemPackage() {
  using ignition::common::SystemPaths;
  std::list<std::string> paths = SystemPaths::PathsFromEnv("DELPHYNE_PACKAGE_PATH");
  DELPHYNE_VALIDATE(!paths.empty(), std::runtime_error,
                    "DELPHYNE_PACKAGE_PATH environment "
                    "variable is not set");
  package_paths_.reserve(paths.size());
  std::copy(paths.begin(), paths.end(), std::back_inserter(package_paths_));
}

ignition::common::URI SystemPackage::DoResolve(const ignition::common::URI& uri) const {
  const std::string uri_scheme = uri.Scheme();
  if (uri_scheme == "file") {
    if (!ignition::common::exists(uri.Path().Str())) {
      return ignition::common::URI();
    }
    return uri;
  }
  if (uri_scheme == "package") {
    using ignition::common::SystemPaths;
    const std::string path = SystemPaths::LocateLocalFile(uri.Path().Str(), package_paths_);
    if (path.empty()) {
      return ignition::common::URI();
    }
    ignition::common::URI newUri(uri);
    newUri.Path() = ignition::common::URIPath(path);
    newUri.SetScheme("file");
    return newUri;
  }
  // TODO(hidmic): Check for existence of other,
  //               likely external, resources too.
  return uri;
}

BundledPackage::BundledPackage(const std::string& path) : path_(path) {}

std::string BundledPackage::ResolveToInternalPath(const ignition::common::URI& uri) const {
  return ignition::common::absPath(ignition::common::joinPaths(path_, uri.Scheme(), uri.Path().Str()));
}

void BundledPackage::DoAdd(const ignition::common::URI& uri) {
  const utility::Package& package_in_use = utility::PackageManager::Instance()->package_in_use();
  const ignition::common::URI resolved_uri = package_in_use.Resolve(uri);
  DELPHYNE_VALIDATE(resolved_uri.Valid(), std::runtime_error, uri.Str() + " resource cannot not be found.");
  DELPHYNE_VALIDATE(resolved_uri.Scheme() == "file", std::runtime_error, uri.Str() + " is not a local resource.");
  const std::string path = resolved_uri.Path().Str();
  const std::string internal_path = ResolveToInternalPath(uri);
  DELPHYNE_VALIDATE(!ignition::common::exists(internal_path), std::runtime_error, uri.Str() + " already exists.");
  const std::string internal_dirpath = Dirname(internal_path);
  if (!ignition::common::exists(internal_dirpath)) {
    DELPHYNE_VALIDATE(ignition::common::createDirectories(internal_dirpath), std::runtime_error,
                      "Cannot setup internal package structure");
  }
  DELPHYNE_VALIDATE(ignition::common::isDirectory(internal_dirpath), std::runtime_error, "Cannot add below resource.");
  DELPHYNE_VALIDATE(ignition::common::copyFile(path, internal_path), std::runtime_error, "Failed to add to package.");
}

ignition::common::URI BundledPackage::DoResolve(const ignition::common::URI& uri) const {
  std::string internal_path = ResolveToInternalPath(uri);
  if (ignition::common::exists(internal_path)) {
    return ignition::common::URI("file://" + internal_path);
  }
  return ignition::common::URI();
}

PackageManager* PackageManager::Instance() {
  static PackageManager instance;
  return &instance;
}

void PackageManager::Use(std::unique_ptr<Package> package) {
  DELPHYNE_VALIDATE(package != nullptr, std::runtime_error, "Cannot make use of a null package.");
  package_ = std::move(package);
}

const Package& PackageManager::package_in_use() const {
  if (!package_) {
    package_ = std::make_unique<SystemPackage>();
  }
  return *package_;
}

}  // namespace utility
}  // namespace delphyne
