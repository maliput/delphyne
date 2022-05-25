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

#include "delphyne/utility/resources/inspection.h"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/common/URI.hh>

#include "delphyne/utility/resources/meshes.h"
#include "delphyne/utility/resources/resources.h"
#include "utility/filesystem.h"

namespace delphyne {
namespace utility {

ResourceInspector* ResourceInspector::Instance() {
  static ResourceInspector instance;
  return &instance;
}

ResourceInspector::ResourceInspector() {
  AssociateExtension("obj", ResourceSubtype<OBJFile>::Instance());
  AssociateExtension("dae", ResourceSubtype<ColladaFile>::Instance());
  AssociateExtension("mtl", ResourceSubtype<MTLFile>::Instance());
}

void ResourceInspector::AssociateExtension(const std::string& extension, const ResourceType* type) {
  DELPHYNE_VALIDATE(type_extension_associations_.count(extension) == 0, std::runtime_error,
                    "Extension '" + extension + "' already associated.");
  DELPHYNE_VALIDATE(type != nullptr, std::logic_error, "Type is null.");
  type_extension_associations_[extension] = type;
}

std::unique_ptr<Resource> ResourceInspector::GetResource(const ignition::common::URI& uri) const {
  // Matches resource name extension with known types.
  std::string extension;
  std::tie(std::ignore, extension) = SplitExtension(uri.Path().Str());
  if (type_extension_associations_.count(extension) == 0) {
    // Unknown type, cannot retrieve resource.
    // TODO(hidmic): Should we perform an exhaustive search over
    //               all known resource types?
    return nullptr;
  }
  // Retrieve resource using the proper type.
  const ResourceType* resource_type = type_extension_associations_.at(extension);
  return resource_type->Instantiate(uri);
}

std::vector<ignition::common::URI> ResourceInspector::GetDependencies(const ignition::common::URI& uri) const {
  std::unique_ptr<Resource> resource = GetResource(uri);
  if (resource == nullptr) return {};
  return resource->GetDependencies();
}

}  // namespace utility
}  // namespace delphyne
