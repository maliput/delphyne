// Copyright 2018 Toyota Research Institute

#include "delphyne/utility/resources/resources.h"

#include <fstream>
#include <regex>
#include <vector>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/common/URI.hh>

#include "common/filesystem.h"
#include "delphyne/utility/package.h"

namespace delphyne {
namespace utility {

Resource::Resource(const ignition::common::URI& uri) : uri_(uri) {
  DELPHYNE_VALIDATE(uri.Valid(), std::runtime_error,
                    "Resource URI is not valid!");
}

std::string Resource::Path() const {
  // Resolves given uri using the current package.
  const utility::Package& package_in_use =
      utility::PackageManager::Instance()->package_in_use();
  const ignition::common::URI resolved_uri = package_in_use.Resolve(Uri());
  // Validates whether we've resolved to a supported resource or not.
  DELPHYNE_VALIDATE(resolved_uri.Valid(), std::runtime_error,
                    "Failed to resolve " + Uri().Str() + ".");
  DELPHYNE_VALIDATE(resolved_uri.Scheme() == "file", std::runtime_error,
                    "Resource is not in the file system.");
  return resolved_uri.Path().Str();
}

GenericResource::GenericResource(const ignition::common::URI& uri,
                                 const std::regex& dependency_pattern)
    : Resource(uri), dependency_pattern_(dependency_pattern) {}

std::vector<ignition::common::URI> GenericResource::GetDependencies() const {
  std::ifstream fs(Path());
  std::vector<ignition::common::URI> dependencies;
  const std::string scheme = Uri().Scheme();
  const std::string dirpath = Dirname(Uri().Path().Str());
  for (std::string line; std::getline(fs, line);) {
    std::smatch match{};
    if (std::regex_search(line, match, dependency_pattern_)) {
      dependencies.push_back(ignition::common::URI(
          scheme + "://" + dirpath + match.str()));
    }
  }
  return dependencies;
}

}  // namespace utility
}  // namespace delphyne
