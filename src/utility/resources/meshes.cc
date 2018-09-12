// Copyright 2018 Toyota Research Institute

#include "delphyne/utility/resources/meshes.h"

#include <regex>
#include <sstream>
#include <string>

namespace delphyne {
namespace utility {

namespace {

// Makes a regular expression to match full file paths with
// the given @p suffix.
std::regex MakePathRegex(const std::string& suffix) {
  static constexpr const char* const kRawPathRegex =
      "([^ <>!\\$`&\\*\\(\\)\\+]|(\\\\[ <>!\\$`&\\*\\(\\)\\+]))+";
  std::stringstream regex_builder{};
  regex_builder << kRawPathRegex << suffix;
  return std::regex(regex_builder.str(),
                    std::regex::extended | std::regex::nosubs);
}

}  // namespace

OBJFile::OBJFile(const ignition::common::URI& uri)
    : GenericResource(uri, MakePathRegex("\\.mtl")) {}

ColladaFile::ColladaFile(const ignition::common::URI& uri)
    : GenericResource(uri, MakePathRegex("\\.(jpg|png)")) {}

MTLFile::MTLFile(const ignition::common::URI& uri)
    : GenericResource(uri, MakePathRegex("\\.(jpg|png)")) {}

}  // namespace utility
}  // namespace delphyne
