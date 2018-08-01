// Copyright 2018 Toyota Research Institute

#include "delphyne/utility/resources.h"

#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/common/URI.hh>

#include "common/filesystem.h"

#include "delphyne/utility/package.h"

namespace delphyne {
namespace utility {

namespace {

bool HasKnownFormat(const std::string& path) {
  return (ignition::common::EndsWith(path, ".obj") ||
          ignition::common::EndsWith(path, ".dae"));
}

// Computes a dependency extraction regular expression for
// the resource at @p path.
std::regex MakeDependencyExpression(const std::string& path) {
  static constexpr const char* const kRawPathPattern =
      "([^ <>!\\$`&\\*\\(\\)\\+]|(\\\\[ <>!\\$`&\\*\\(\\)\\+]))+";
  std::stringstream expression_builder{};
  if (ignition::common::EndsWith(path, ".obj")) {
    expression_builder << kRawPathPattern << "\\.mtl";
  } else if (ignition::common::EndsWith(path, ".dae")) {
    expression_builder << kRawPathPattern << "\\.(jpg|png)";
  }
  return std::regex(expression_builder.str(),
                    std::regex::extended | std::regex::nosubs);
}

}  // namespace

ResourceInspector* ResourceInspector::Instance() {
  static ResourceInspector instance;
  return &instance;
}

std::vector<ignition::common::URI>
ResourceInspector::Depends(const ignition::common::URI& uri) const {
  // TODO(hidmic): Limited support for mesh file formats only.
  // To be generalized in a follow-up work.

  // Resolves given uri using the current package.
  const utility::Package& package_in_use =
      utility::PackageManager::Instance()->package_in_use();
  const ignition::common::URI resolved_uri = package_in_use.Resolve(uri);
  DELPHYNE_VALIDATE(resolved_uri.Valid(), std::runtime_error,
                    "Failed to resolve " + uri.Str() + ".");
  DELPHYNE_VALIDATE(resolved_uri.Scheme() == "file", std::runtime_error,
                    resolved_uri.Str() + " is not supported.");
  const std::string path = "/" + resolved_uri.Path().Str();
  std::vector<ignition::common::URI> dependencies{};
  if (HasKnownFormat(path)) {
    // Extracts dependencies from resource.
    std::ifstream fs(path);
    std::regex pattern = MakeDependencyExpression(path);
    for (std::string line; std::getline(fs, line);) {
      std::smatch match{};
      if (std::regex_search(line, match, pattern)) {
        dependencies.push_back(ignition::common::URI(
            uri.Scheme() + "://" + ignition::common::joinPaths(
                Dirname("/" + uri.Path().Str()), match.str())));
      }
    }
  }
  return dependencies;
}

}  // namespace utility
}  // namespace delphyne
