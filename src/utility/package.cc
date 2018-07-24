// Copyright 2018 Toyota Research Institute

#include "delphyne/utility/package.h"

#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/URI.hh>

#include "delphyne/macros.h"

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
  std::list<std::string> paths =
      SystemPaths::PathsFromEnv("DELPHYNE_PACKAGE_PATH");
  if (!paths.empty()) {
    package_paths_.reserve(paths.size());
    std::copy(paths.begin(), paths.end(),
              std::back_inserter(package_paths_));
  } else {
    ignerr << "DELPHYNE_PACKAGE_PATH environment "
           << "variable is not set" << std::endl;
  }
}

std::string
SystemPackage::DoFind(const ignition::common::URI& uri) const {
  std::string path = "";
  const std::string uri_scheme = uri.Scheme();
  const std::string uri_path = uri.Path().Str();
  if (uri_scheme == "file") {
    // TODO(hidmic): Fix bug in ignition common
    // when dealing with absolute URI paths (e.g.
    // file:///tmp/test)
    if (ignition::common::exists("/" + uri_path)) {
      path = "/" + uri_path;
    }
  } else if (uri_scheme == "package") {
    using ignition::common::SystemPaths;
    path = SystemPaths::LocateLocalFile(
        uri_path, package_paths_);
  }
  return path;
}

BundledPackage::BundledPackage(const std::string& path)
    : path_(path) {}

std::string BundledPackage::ResolveToInternalLocation(
    const ignition::common::URI& uri) const {
  using ignition::common::absPath;
  using ignition::common::joinPaths;
  return absPath(joinPaths(path_, uri.Scheme(), uri.Path().Str()));
}

bool BundledPackage::Add(const ignition::common::URI& uri,
                         const std::string& external_path) {
  if (!uri.Valid()) return false;
  const std::string internal_path = ResolveToInternalLocation(uri);
  const std::string internal_dirpath =
      internal_path.substr(0, internal_path.rfind("/"));
  if (!ignition::common::exists(internal_dirpath)) {
    ignition::common::createDirectories(internal_dirpath);
  }
  return (ignition::common::isDirectory(internal_dirpath) &&
          ignition::common::copyFile(external_path, internal_path));
}

std::string BundledPackage::DoFind(const ignition::common::URI& uri) const {
  const std::string internal_path = ResolveToInternalLocation(uri);
  if (ignition::common::exists(internal_path)) return internal_path;
  return "";
}

PackageManager* PackageManager::Instance() {
  static PackageManager instance;
  return &instance;
}

void PackageManager::Use(std::unique_ptr<Package> package) {
  DELPHYNE_VALIDATE(package != nullptr, std::runtime_error,
                    "Cannot make use of a null package.");
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
