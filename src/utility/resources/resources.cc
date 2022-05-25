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

#include "delphyne/utility/resources/resources.h"

#include <fstream>
#include <regex>
#include <vector>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/common/URI.hh>

#include "delphyne/utility/package.h"
#include "utility/filesystem.h"

namespace delphyne {
namespace utility {

Resource::Resource(const ignition::common::URI& uri) : uri_(uri) {
  DELPHYNE_VALIDATE(uri.Valid(), std::runtime_error, "Resource URI is not valid!");
}

std::string Resource::Path() const {
  // Resolves given uri using the current package.
  const utility::Package& package_in_use = utility::PackageManager::Instance()->package_in_use();
  const ignition::common::URI resolved_uri = package_in_use.Resolve(Uri());
  // Validates whether we've resolved to a supported resource or not.
  DELPHYNE_VALIDATE(resolved_uri.Valid(), std::runtime_error, "Failed to resolve " + Uri().Str() + ".");
  DELPHYNE_VALIDATE(resolved_uri.Scheme() == "file", std::runtime_error, "Resource is not in the file system.");
  return resolved_uri.Path().Str();
}

GenericResource::GenericResource(const ignition::common::URI& uri, const std::regex& dependency_pattern)
    : Resource(uri), dependency_pattern_(dependency_pattern) {}

std::vector<ignition::common::URI> GenericResource::GetDependencies() const {
  std::ifstream fs(Path());
  std::vector<ignition::common::URI> dependencies;
  const std::string scheme = Uri().Scheme();
  const std::string dirpath = Dirname(Uri().Path().Str());
  for (std::string line; std::getline(fs, line);) {
    std::smatch match{};
    if (std::regex_search(line, match, dependency_pattern_)) {
      dependencies.push_back(ignition::common::URI(scheme + "://" + dirpath + match.str()));
    }
  }
  return dependencies;
}

}  // namespace utility
}  // namespace delphyne
